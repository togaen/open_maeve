require(latex2exp)

# Be sure working directory is set to source file location: Session > Set Working Directory
source('constraint-functions.R')

# plot_constraint("hard",0,0,1,-3,3,1,1,5,c(-25, 0))
# plot_constraint("soft",0,0,1,-5,7,1,0.3,5,c(0,1))
plot_constraint <- function(f_type, t, r_min, r_max, from_s, to_s, alpha, beta, steps, y_range) {
  quartz(width=9, height=5) # for Mac platform
  x = get_range(from_s,to_s)
  x_range = range(x)
  x_range[1] = x_range[1] + 0.2
  x_range[2] = x_range[2] - 0.2
  
  par(oma=c(0,1,0,0))
  plot(NULL, xlab='input value', ylab='potential value', xlim=x_range, ylim=y_range,type='l',cex.lab=1.5)
  
  if (f_type == "soft") {
    f = soft_constraint
  } else if (f_type == "hard") {
    f = hard_constraint
  }
  y = sapply(x, function(s) return(f(s, t, r_min, r_max, alpha, beta)))
  lines(x, y, col=hsv(0.333,0.25,0.75),lwd=1) # green
  
  # draw threshold
  if (f_type == "soft") {
    x_thresh = get_range(x_range[1]-1,x_range[2]+1)
    y_min_thresh = rep(r_min, length(x_thresh))
    y_max_thresh = rep(r_max, length(x_thresh))
    lines(x_thresh, y_min_thresh, col=hsv(0, 0.5, 0.5), lty=3)
    lines(x_thresh, y_max_thresh, col=hsv(0, 0.5, 0.5), lty=2)
  } else if (f_type == "hard") {
    y_thresh = get_range(y_range[1]-1,y_range[2]+1)
    x_min_thresh = rep(r_min, length(y_thresh))
    x_max_thresh = rep(r_max, length(y_thresh))
    lines(x_min_thresh, y_thresh, col=hsv(0, 0.5, 0.5), lty=3)
    lines(x_max_thresh, y_thresh, col=hsv(0, 0.5, 0.5), lty=2)
  }
}

get_range <- function(from_val, to_val, steps=200) {
  return(seq(from_val, to_val, by=((to_val-from_val)/steps)))
}

# Soft plot: plot_constraint_range("soft",0,0,1,-5,7,1,5,0.3,1.3,5,c(0,1),-5,0.9)
# Hard plot: plot_constraint_range("hard",0,0,1,-3,3,1,7,1,7,5,c(-25, 0),-2.9,-8)
plot_constraint_range <- function(f_type, t, r_min, r_max, from_s, to_s, from_alpha, to_alpha, from_beta, to_beta, steps, y_range, legend_x, legend_y) {
  quartz(width=9, height=5) # for Mac platform
  alpha_range = get_range(from_alpha,to_alpha, steps)
  beta_range = get_range(from_beta,to_beta, steps)
  x = get_range(from_s,to_s)
  x_range = range(x)
  x_range[1] = x_range[1] + 0.2
  x_range[2] = x_range[2] - 0.2
  
  color_min = 0.25
  color_max = 0.75
  
  par(oma=c(0,1,0,0))
  plot(NULL, xlab='input value', ylab='potential value', xlim=x_range, ylim=y_range,type='l',cex.lab=1.5)
  
  # get constraint function
  if (f_type == "soft") {
    f = soft_constraint
  } else if (f_type == "hard") {
    f = hard_constraint
  }
  
  c = color_min
  alpha_color_step = (color_max - color_min) / length(alpha_range)
  for (beta in beta_range) {
    y = sapply(x, function(s) return(f(s, t, r_min, r_max, 1, beta)))
    lines(x, y, col=hsv(0.666,c,0.75),lwd=1) # blue
    c = c + alpha_color_step
  }
  
  c = color_min
  beta_color_step = (color_max - color_min) / length(beta_range)
  for (alpha in alpha_range) {
    y = sapply(x, function(s) return(f(s, t, r_min, r_max, alpha, 1)))
    lines(x, y, col=hsv(0.333,c,0.75),lwd=1) # green
    c = c + beta_color_step
  }
  
  # draw threshold
  if (f_type == "soft") {
    x_thresh = get_range(x_range[1]-1,x_range[2]+1)
    y_min_thresh = rep(r_min, length(x_thresh))
    y_max_thresh = rep(r_max, length(x_thresh))
    lines(x_thresh, y_min_thresh, col=hsv(0, 0.5, 0.5), lty=3)
    lines(x_thresh, y_max_thresh, col=hsv(0, 0.5, 0.5), lty=2)
  } else if (f_type == "hard") {
    y_thresh = get_range(y_range[1]-1,y_range[2]+1)
    x_min_thresh = rep(r_min, length(y_thresh))
    x_max_thresh = rep(r_max, length(y_thresh))
    lines(x_min_thresh, y_thresh, col=hsv(0, 0.5, 0.5), lty=3)
    lines(x_max_thresh, y_thresh, col=hsv(0, 0.5, 0.5), lty=2)
  }
  
  # draw legend
  alpha_color = hsv(0.333,1,0.75)
  beta_color = hsv(0.666,1,0.75)
  tex_alpha_label = paste('$\\alpha$=$',from_alpha,'\\ldots,',to_alpha,'$')
  tex_beta_label = paste('$\\beta$=$',from_beta,',\\ldots,',to_beta,'$')
  legend(legend_x, legend_y, cex=1.5,legend=c(TeX(tex_alpha_label), TeX(tex_beta_label), TeX('$c_{max}$'), TeX('$c_{min}$')), lty=c(1,1,2,3), lwd=c(2,1,1,1),col=c(alpha_color,beta_color,hsv(0, 0.5, 0.5),hsv(0,0.5,0.5)))  

  # write to file
  if (f_type == "hard") {
    dev.copy2pdf(device=xquartz, file = "C_h.pdf")
  } else if (f_type == "soft") {
    dev.copy2pdf(device=xquartz, file = "C_s.pdf")
  }
}