#include <core/gnu_multi_plot.h>

int main()
{
  GnuMultiPlot plot;

  for (int i = 0; i < 100; ++i)
  {
    plot.add("curve A", "i**2", i, i * i);
    plot.add("curve A", "i", i, i);
    plot.add("curve A", "i*5", i, i * 5);
    plot.add("curve B", "i**3", i, i * i * i);
    plot.add("curve C", "i**4", i, i * i * i * i);
    plot.render();
    usleep(150000);
  }
  plot.closeWindow();
}
