///////////////////////////////////////////////////////////////////////////////
// The roscpp package provides a c++ implementation for ROS.
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <csignal>
#include <cstring>
#include "console_master.h"
#include "topic.h"
#include "service.h"

bool no_control_c = true;
void sig_handler(int sig)
{
  no_control_c = false;
}

console_master::console_master(int port)
: master(port)
{
#ifdef USE_CURSES
  initscr();
  cbreak();
  keypad(stdscr, TRUE);
  noecho();
  refresh();
  int h, w;
  getmaxyx(stdscr, h, w);
  main_win_height = h-10;
  log_win =  newwin(             10, w, main_win_height, 0);
  main_win = newwin(main_win_height, w,               0, 0);
  scrollok(log_win, TRUE);
  gui_ok = true;
  refresh_gui();
#else
  gui_ok = true;
  printf("BOTHERDER: All your computer are belong to us.\n");
#endif
}

void console_master::log_console(log_level_t level, const char *format, 
                                 va_list args) 
{
#ifdef USE_CURSES
  /*
  if (log_win)
  {
    wprintw(log_win, "> ");
    vwprintw(log_win, format, args);
    if (format[strlen(format)-1] != '\n')
      wprintw(log_win, "\n");
    refresh_gui();
  }
  */
  refresh_gui();
#else
  vprintf(format, args);
  if (format[strlen(format)-1] != '\n')
    printf("\n");
#endif
}

void console_master::_refresh_gui()
{
#ifdef USE_CURSES
  int h, w;
  getmaxyx(stdscr, h, w);
  wrefresh(log_win);
  werase(main_win);
  wmove(main_win, 0, 0);
  waddstr(main_win, "BOTHERDER. All your computer are belong to us.");
  mvwhline(main_win, main_win_height-1, 0, 0, w);
  wmove(main_win, 2, 0);
  if (topics.size() == 0)
    wprintw(main_win, "no topics.\n");
  else if (topics.size() == 1)
    wprintw(main_win, "%d topic:\n", topics.size());
  else
    wprintw(main_win, "%d topics:\n", topics.size());

  for (vector<topic *>::iterator t = topics.begin(); t != topics.end(); ++t)
  {
    wprintw(main_win, "  %s (%d pub, %d sub)\n", (*t)->name.c_str(), 
      (*t)->pubs.size(), (*t)->subs.size());
  }
  wprintw(main_win, "\n");

  if (services.size() == 0)
    wprintw(main_win, "no services.\n");
  else if (services.size() == 1)
    wprintw(main_win, "1 service:\n");
  else
    wprintw(main_win, "%d services:\n", services.size());
  for (vector<service *>::iterator s = services.begin(); 
       s != services.end(); ++s)
  {
    wprintw(main_win, "  %s (pub: %s, %d subs)\n", (*s)->name.c_str(),
            ((*s)->pub ? (*s)->pub_uri.c_str() : ""), (*s)->subs.size());
  }
  wprintw(main_win, "\n");

  if (procs.size() == 0)
    wprintw(main_win, "no processes.\n");
  else if (procs.size() == 1)
    wprintw(main_win, "1 process:\n");
  else
    wprintw(main_win, "%d processes:\n", procs.size());
  for (vector<process *>::iterator p = procs.begin(); p != procs.end(); ++p)
  {
    int nt = num_topics(*p);
    wprintw(main_win, "  %s at %s (%d topic%s)\n", (*p)->name.c_str(), 
      (*p)->uri.c_str(), nt, (nt == 1 ? "" : "s"));
  }

  wprintw(main_win, "\n");
  if (params.size() == 0)
    wprintw(main_win, "no params.\n");
  else if (params.size() == 1)
    wprintw(main_win, "1 param:\n");
  else
    wprintw(main_win, "%d params:\n", params.size());

  for (map<string, XmlRpcValue>::iterator p = params.begin();
       p != params.end(); ++p)
  {
    wprintw(main_win, "  %s ", (*p).first.c_str());
    // just handle the primitive parameter types for now
    if (p->second.getType() == XmlRpcValue::TypeString)
    {
      // truncate it so it isn't forever long on the screen
      const size_t MAX_PARAM_DISPLAY_LEN = 30;
      string s = string(string(p->second), 0, MAX_PARAM_DISPLAY_LEN);
      // don't display any newlines or other evil characters
      for (size_t i = 0; i < s.length(); i++)
        if (iscntrl(s[i])) 
          s[i] = ' ';
      // finally, dump the hacked-up string to the GUI
      wprintw(main_win, "(string) = %s%s\n", s.c_str(),
              s.length() == MAX_PARAM_DISPLAY_LEN ? "[...]" : "");
    }
    else if (p->second.getType() == XmlRpcValue::TypeDouble)
      wprintw(main_win, "(double) = %f\n", double(p->second));
    else if (p->second.getType() == XmlRpcValue::TypeInt)
      wprintw(main_win, "(int) = %d\n", int(p->second));
    else if (p->second.getType() == XmlRpcValue::TypeBoolean)
    {
      wprintw(main_win, "(bool) ");
      if ((bool)(p->second) == true)
        wprintw(main_win, "true\n");
      else
        wprintw(main_win, "false\n");
    }

  }

  wrefresh(main_win);
#endif
}

void console_master::spin()
{
  signal(SIGINT, sig_handler);
  while (!shutting_down && no_control_c)
    usleep(10000);
  log(INFO, "blowing up...\n");
  self_destruct();
  log(INFO, "done blowing up. shutting down ncurses...\n");
  while (_ok)
    usleep(10000);
#ifdef USE_CURSES
  delwin(main_win);
  delwin(log_win);
  log_win = NULL;
  // not sure why valgrind says there is still memory left over. perhaps
  // there is another step to shutting down ncurses?
  endwin();
#endif
}

