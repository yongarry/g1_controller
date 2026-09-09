#pragma once

#include <string>
#include <vector>
#include <deque>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <cstdlib>
#include <sys/select.h>
#include <iostream>
#include <thread>


/**
 * @brief Maintain a keyboard reading thread.
 * And get the latest key value.
 *
 * Reads /dev/tty (the controlling terminal), not stdin, so keys still work
 * if stdin is a pipe. The process must be in the foreground process group
 * or the kernel will stop it with SIGTTIN.
 */
class Keyboard
{
public:
  Keyboard()
  {
    _fd = open("/dev/tty", O_RDONLY);
    if (_fd < 0) _fd = fileno(stdin);

    if (tcgetattr(_fd, &_oldSettings) == 0)
    {
      _newSettings = _oldSettings;
      _oldSettings.c_lflag |= (ICANON | ECHO);
      _newSettings.c_lflag &= (~ICANON & ~ECHO);
      _have_termios = true;
      s_restore_fd = _fd;
      s_restore_settings = _oldSettings;
      std::atexit(Keyboard::restore_atexit);
      std::signal(SIGINT, Keyboard::signal_restore);
      std::signal(SIGTERM, Keyboard::signal_restore);
    }

    _startKey();

    _thread_running = true;
    _readThread = std::thread([this] {
      while (_thread_running) {
        _read();
      }
    });
  }

  ~Keyboard()
  {
    _thread_running = false;
    _pauseKey();
    if (_readThread.joinable()) _readThread.join();
    restore_tty(_fd, &_oldSettings);
    if (_fd >= 0 && _fd != fileno(stdin)) close(_fd);
  }

  bool has_tty() const { return _have_termios; }

  void update()
  {
    if(_key != _last_key)
    {
      on_pressed = _key != "";
      on_released = _key == "";
    }
    else
    {
      on_pressed = false;
      on_released = false;
    }
    
    _last_key = _key;
  }

  /**
   * @brief Get the current key value
   * 
   * @return std::string 
   */
  std::string key() const { return _key; };

  // Overlay WASD/QE onto Unitree stick axes (ly forward, lx right, rx yaw-right).
  // Returns true if a teleop key is currently held.
  bool apply_stick_axes(float& ly, float& lx, float& rx) const
  {
    if (_key == "w") { ly =  1.f; lx =  0.f; rx =  0.f; return true; }
    if (_key == "s") { ly = -1.f; lx =  0.f; rx =  0.f; return true; }
    if (_key == "a") { ly =  0.f; lx = -1.f; rx =  0.f; return true; }
    if (_key == "d") { ly =  0.f; lx =  1.f; rx =  0.f; return true; }
    if (_key == "q") { ly =  0.f; lx =  0.f; rx = -1.f; return true; }
    if (_key == "e") { ly =  0.f; lx =  0.f; rx =  1.f; return true; }
    return false;
  }

  /**
   * @brief Get the String object from keyboard 
   * 
   * @param slogan Used to prompt the user for input
   * @return std::string 
   */
  std::string getString(std::string slogan)
  {
    // Stop reading keyboard value
    _running = false;
    _pauseKey();

    std::string stringtemp;
    std::cout << slogan << std::endl;// prompt
    std::getline(std::cin, stringtemp);

    // Restart reading keyboard value
    _startKey();
    _running = true;

    return stringtemp;
  }

  /**
   * flags; available after update()
   */
  bool on_pressed = false;
  bool on_released = false;

  private:
  bool _thread_running = false;
  bool _running = false;
  bool _have_termios = false;
  int _fd = -1;
  std::thread _readThread;

  static inline int s_restore_fd = -1;
  static inline termios s_restore_settings{};

  static void restore_tty(int fd, const termios* settings)
  {
    if (fd >= 0 && settings) tcsetattr(fd, TCSANOW, settings);
  }

  static void restore_atexit()
  {
    restore_tty(s_restore_fd, &s_restore_settings);
  }

  static void signal_restore(int sig)
  {
    restore_atexit();
    std::signal(sig, SIG_DFL);
    raise(sig);
  }

  void _read()
  {
    if(_running)
    {
      FD_ZERO(&_fd_set);
      FD_SET(_fd, &_fd_set);

      _tv.tv_sec = 0;
      _tv.tv_usec = 80000;

      if(select(_fd+1, &_fd_set, NULL, NULL, &_tv) > 0)
      {
        // Read the key value into _c
        int res = read(_fd, &_c, 1);
        if (res <= 0) { _key = ""; return; }

        // Parser the key value
        if(_c != '\033') {
          // This is a normal key
          _key = _c;
        }else{
          // This is a special key
          int m = read(_fd, &_c, 1);
          if(_c == '[')
          {
            m = read(_fd, &_c, 1);
            switch (_c)
            {
            case 'A': _key = "up";    break;
            case 'B': _key = "down";  break;
            case 'C': _key = "right"; break;
            case 'D': _key = "left";  break;
            default:  _key = "";      break;
            }
          }
        }
      }else{
        _key = "";
      }
    }
    else
    {
      usleep(20000);
    }
  }

  /**
   * @brief Restore keyboard default settings.
   */
  void _pauseKey()
  {
    if (_have_termios) tcsetattr(_fd, TCSANOW, &_oldSettings);
    _running = false;
  }

  /**
   * @brief Disable canonical mode and echoing of input characters.
   */
  void _startKey()
  {
    if (_have_termios) tcsetattr(_fd, TCSANOW, &_newSettings);
    _running = true;
  }

  fd_set _fd_set;
  char _c = '\0';
  std::string _key, _last_key;
  
  termios _oldSettings, _newSettings;
  timeval _tv;
};
