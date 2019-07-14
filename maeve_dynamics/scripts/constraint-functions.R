
#
# Note: variables names in these functions may not match what's in the paper.
#

soft_constraint <- function(s, t_x, r_min, r_max, alpha=1, beta=1) {
  r_mid = (r_max + r_min) / 2
  return(r_min + (r_max - r_min) / ((1 + exp(-alpha * (s - t_x - r_mid))))^(1/beta))
}

hard_constraint <- function(s, t_y, r_min, r_max, alpha=1, beta=1) {
  if (s < r_min) {
    return(t_y - alpha/((r_min - s)^beta))
  }
  if (s > r_max) {
    return(t_y - alpha/((s - r_max)^beta))
  }
  return(-Inf)
}