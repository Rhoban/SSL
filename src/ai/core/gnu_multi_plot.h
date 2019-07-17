#pragma once

#include <map>
#include <vector>
#include <deque>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>

/**
 * Plot
 *
 * Programming interface with
 * the Linux ploting utility
 * Gnuplot using VectorLabel
 */
class GnuMultiPlot
{
  std::map<std::string, std::map<std::string, std::deque<std::pair<double, double>>>> datas;
  int displayed_values_;

public:
  GnuMultiPlot();
  ~GnuMultiPlot();

  void setDisplayedValues(int size);

  void add(std::string plot_name, std::string curve_name, double x, double y);

  /**
   * Render the plot
   * Wait until plot window is closed
   */
  void render();

  /**
   * Close gnuplot
   */
  void closeWindow();

  void clear();

private:
  /**
   * Gnuplot pipe file descriptor
   */
  int plotFd;

  /**
   * Values to plot
   */

  /**
   * Fork current process to create a new GnuPlot window
   */
  void createGnuplotInstance();

  /**
   * Wait for end of gnuplot session and
   * close the openned pipe to GnuPlot window instance
   */
  void waitCloseGnuplotInstance();

  /**
   * Generate and return Gnuplot commands and data
   */
  std::string generatePlotting();
};
