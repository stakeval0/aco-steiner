
#######################################################################
#
# utilities.R - various utilities and routines
#
# Copyright (c) 2006 by Krzysztof Socha (ksocha@ulb.ac.be)
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by 
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#

# Generates randomized n-dimensional rotation matrix
gen.R <- function(n) {
  # generate n, n-dimensional vectors
  vec <- runif(n^2,-1,1)
  dim(vec) <- c(n,n)
  # calculate the orthogonal rotation matrix
  R <- qr.Q(qr(vec), complete=TRUE) # rot. matrix after orthogonalization   
  if (det(R)<0) {
    R[,1] <- -R[,1]
  }
  return(R)
}

# Produces aligned seqence - so that at least one element is equal to 0
seq.a <- function(from,to,length.out) {
	if (length.out==1)
		return(0)
	s <- seq(from,to,length.out=length.out)
	if (min(abs(s))>0) {
		s <- seq(from,to,length.out=length.out+1)
		s <- s-s[which.min(abs(s))]
		if (from<to) 
			s <- s[s>=from & s<=to]
		else
			s <- s[s>=to & s<=from]
		if (length(s)>length.out) {
			if (abs(from)<abs(to))
				s <- s[1:length.out]
			else
				s <- s[2:(length.out+1)]
		}
	}
	return(s)
}

# Check if required global variables have been defined
# and if not - define default initial values

# deformation = 100
if (exists("deformation",mode="numeric")) {
	cat(c("Already defined:   deformation =",deformation,"\n"))
} else {
	assign("deformation", 100, env = .GlobalEnv)
	cat(c("Asssuming default: deformation =",deformation,"\n"))
}

# resolution = 10
if (exists("resolution",mode="numeric")) {
	cat(c("Already defined:   resolution =",resolution,"\n"))
} else {
	assign("resolution", 10, env = .GlobalEnv)
	cat(c("Asssuming default: resolution =",resolution,"\n"))
}

# max.intercepts = 1
if (exists("max.intercepts",mode="numeric")) {
	cat(c("Already defined:   max.intercepts =",max.intercepts,"\n"))
} else {
	assign("max.intercepts", 1, env = .GlobalEnv)
	cat(c("Asssuming default: max.intercepts =",max.intercepts,"\n"))
}

# T.cold = 4.2
if (exists("T.cold",mode="numeric")) {
	cat(c("Already defined:   T.cold =",T.cold,"\n"))
} else {
	assign("T.cold", 4.2, env = .GlobalEnv)
	cat(c("Asssuming default: T.cold =",T.cold,"\n"))
}

# T.hot = 300
if (exists("T.hot",mode="numeric")) {
	cat(c("Already defined:   T.hot =",T.hot,"\n"))
} else {
	assign("T.hot", 300, env = .GlobalEnv)
	cat(c("Asssuming default: T.hot =",T.hot,"\n"))
}
