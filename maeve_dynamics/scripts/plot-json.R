library("rjson")
# json_data <- fromJSON(file='~/Desktop/test.json')

parabola <- function(a, b, c, value) {
  return(a * value * value + b * value + c);
}

plot_connector <- function(legend_x, legend_y, connector) {
  t1 = connector$parabolic_segments[[1]]$domain$min
  t2 = connector$parabolic_segments[[2]]$domain$min
  t3 = connector$parabolic_segments[[3]]$domain$min
  t4 = connector$parabolic_segments[[3]]$domain$max
  x = seq(t1, t4, 0.01)
  y = c();
  
  for (t in x) {
    if (t < t2) {
      y = c(y, parabola(connector$parabolic_segments[[1]]$a,connector$parabolic_segments[[1]]$b, connector$parabolic_segments[[1]]$c, t))
    }
    else if (t < t3) {
      y = c(y, parabola(connector$parabolic_segments[[2]]$a,connector$parabolic_segments[[2]]$b, connector$parabolic_segments[[2]]$c, t))
    }
    else {
      y = c(y, parabola(connector$parabolic_segments[[3]]$a,connector$parabolic_segments[[3]]$b, connector$parabolic_segments[[3]]$c, t))
    }
  }

  #print(x)
  #print(y)
  
  w = 9
  h = 5
  quartz(width=w, height=h) # for Mac platform

  par(oma=c(0,1,0,0))
  #pdf(file="~/Desktop/figure.pdf",width=w,height=h)
  plot(NULL, xlab='Time (s)', cex=1.25, main='Connecting Trajectory between (0.0, 0.0) and (1.0, 0.5)',ylab='Path Position (m)', xlim=range(x), ylim=range(y), type='l',cex.lab=1.5)
  
  lines(x, y, col=hsv(0.666,0.75,0.75),lwd=2)
  #dev.off()
  #legend(legend_x, legend_y, cex=1.25, legend=c('Upper Bound'), lty=c(1), lwd=c(2),col=c(hsv(0.333,0.75,0.75)))
}

plot_intervals <- function(legend_x, legend_y, intervals) {
  x = c();
  y_mins = c();
  y_maxs = c();
  for (interval in intervals) {
    x = c(x, interval$p);
    y_mins = c(y_mins, interval$interval$min)
    y_maxs = c(y_maxs, interval$interval$max)
  }
  
  w = 9
  h = 5
  quartz(width=w, height=h) # for Mac platform
  
  range_x = range(x)
  range_y_min = range(y_mins)
  range_y_max = range(y_maxs)
  
  par(oma=c(0,1,0,0))
  
  #pdf(file="~/Desktop/figure.pdf",width=w,height=h)
  plot(NULL, xlab='Path Position (m)', cex=1.25, main='Reachable Speeds after 1 Second',ylab='Reachable Speed (m/s)', xlim=range_x, ylim=c(range_y_min[1], range_y_max[2]), type='l',cex.lab=1.5)
  
  lines(x, y_mins, col=hsv(0.666,0.75,0.75),lwd=2)
  lines(x, y_maxs, col=hsv(0.333,0.75,0.75),lwd=2)
  
  legend(legend_x, legend_y, cex=1.25, legend=c('Upper Bound', 'Lower Bound'), lty=c(1,1), lwd=c(2,2),col=c(hsv(0.333,0.75,0.75),hsv(0.666,0.75,0.75)))

  #dev.off()
}