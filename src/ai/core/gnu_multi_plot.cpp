// #define PLOT_ACCESSIBILITY_MODE
#include "gnu_multi_plot.h"
#include <assert.h>

GnuMultiPlot::GnuMultiPlot() : displayed_values_(20), plotFd(-1)
{
}

GnuMultiPlot::~GnuMultiPlot()
{
}

void GnuMultiPlot::setDisplayedValues(int size)
{
  assert(size > 0);
  displayed_values_ = size;
}

void GnuMultiPlot::add(std::string plot_name, std::string curve_name, double x, double y)
{
  auto plot = datas.find(plot_name);
  if (plot == datas.end())
  {
    datas[plot_name];
    plot = datas.find(plot_name);
  }
  auto curve = plot->second.find(curve_name);
  if (curve == plot->second.end())
  {
    plot->second[curve_name];
    curve = plot->second.find(curve_name);
  }
  curve->second.push_back(std::pair<double, double>(x, y));
}

void GnuMultiPlot::render()
{
  std::string commands = generatePlotting();

  // std::cout << "gnumultiplot commands is:" << std::endl << "--" << std::endl << commands << "--" << std::endl;

  // Send to gnuplot instance
  if (plotFd <= 0)
  {
    createGnuplotInstance();
  }
  int result = write(plotFd, commands.c_str(), commands.length());
  if (result != (int)commands.length())
  {
    std::cerr << "GnuPlot::render: failed write" << std::endl;
  }
  //    replot = true;
  //    waitCloseGnuplotInstance();
}

std::string GnuMultiPlot::generatePlotting()
{
  std::string commands;
  // std::string data;
  char buffer[100];

  commands = "";
  commands += "set term qt noraise noenhanced; \n";
  int nplots = datas.size();
  commands += "set multiplot layout ";
  if (nplots > 3)
    snprintf(buffer, 100, "%d,%d", nplots / 3 + 1, 3);
  else
    snprintf(buffer, 100, "%d,%d", 1, nplots);
  commands += buffer;
  commands += "\n";
  for (auto plot : datas)
  {
    bool isFirst = true;
    commands += "set xrange[*:*]\n";
    commands += "set yrange[*:*]\n";
    commands += "set ytics auto\n";
    commands += "set xtics auto\n";
    // snprintf(buffer, 100, "%s", nplots);
    commands += "set title '";
    commands += plot.first;
    commands += "'\n";
    commands += "plot ";
    for (auto curve : plot.second)
    {
      if (!isFirst)
      {
        commands += ", ";
      }
      isFirst = false;
      commands += "'-' u 1:2 ";
      commands += " w l";
      commands += " title '";
      //      snprintf(buffer, 100, "curve %d", n);
      //      commands += buffer;
      commands += curve.first;
      commands += "' ";
    }

    for (auto curve : plot.second)
    {
      std::ostringstream oss;
      while (curve.second.size() > displayed_values_)
        curve.second.pop_front();
      for (auto values : curve.second)
      {
        oss << values.first << " " << values.second << std::endl;
      }
      commands += oss.str();
      commands += "e\n";
    }
    // commands += ";\n";
  }
  commands += "unset multiplot\n";
  return commands;
}

void GnuMultiPlot::waitCloseGnuplotInstance()
{
  waitpid(-1, NULL, 0);
  if (plotFd != -1)
  {
    close(plotFd);
    plotFd = -1;
  }
}

void GnuMultiPlot::createGnuplotInstance()
{
  // Creating communication pipe
  int pipefd[2];
  if (pipe(pipefd) == -1)
  {
    throw std::runtime_error("Plot failed to create pipe");
  }

  // Forkink current process
  pid_t pid = fork();
  if (pid > 0)
  {
    // Closing reading pipe end
    close(pipefd[0]);
    // Saving pipe fd
    plotFd = pipefd[1];
  }
  else if (pid == 0)
  {
    // Closing writting pipe end
    close(pipefd[1]);
    // Redirecting reading pipe end to standart input
    if (dup2(pipefd[0], STDIN_FILENO) == -1)
    {
      throw std::runtime_error("Plot failed to dup2");
    }
    // Closing err
    int null = open("/dev/null", O_WRONLY);
    if (dup2(null, STDOUT_FILENO) == -1)
    {
      throw std::runtime_error("Plot failed to dup2");
    }
    int fderr = dup(STDERR_FILENO);
    if (dup2(null, STDERR_FILENO) == -1)
    {
      throw std::runtime_error("Plot failed to dup2");
    }
    // Calling Gnuplot
    execlp("gnuplot", "gnuplot", "-", NULL);
    dup2(fderr, STDERR_FILENO);
    std::cerr << "RhIOShell: Unable to create gnuplot process" << std::endl;
    std::cerr << "Have you installed the gnuplot package ?" << std::endl;
    throw std::logic_error("GnuPlot exec fails");
  }
  else
  {
    throw std::runtime_error("Plot failed to fork");
  }
}

void GnuMultiPlot::closeWindow()
{
  if (write(plotFd, "quit\n", 5) != 5)
    std::cerr << "GnuPlot::closeWindow: failed to quit" << std::endl;
  if (write(plotFd, "quit\n", 5) != 5)
    std::cerr << "GnuPlot::closeWindow: failed to quit" << std::endl;
  if (close(plotFd) != 0)
  {
    perror("GnuPlot::closeWindow: failed to close plotFd");
  }
}

void GnuMultiPlot::clear()
{
  datas.clear();
}
