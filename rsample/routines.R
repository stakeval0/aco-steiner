#######################################################################
#
# routines.R - R implementation of the core routines of the ACO_MV and
#              ACO_R algorithms for continuous and mixed-variable 
#              optimization problems.
#
# Copyright (c) 2006 by Krzysztof Socha (ksocha@ulb.ac.be)
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by 
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#

# Calculates distance according to Euclidian metric
euc.dist <- function(d) {
  return(sqrt(sum(d^2)))
}

# Finds the list of points to be removed from an archive 
# according to certain crowding rule.
# Note: returns the logical vector of points that STAY!
trunc.p.random <- function(p,p.X,size) {
  n <- length(p$v)-size;          # identify the number of points to be removed
  index <- rep(TRUE,length(p$v))  # create the initial logical vector
  if (size==0) return(!index)
  idx <- sample(length(p$v),n,prob=p$gr)
  index[idx] <- FALSE
  return(index)
}

# Generates a set of values for continuous variables based on given distributions  
# (with probability of using a distribution based on its rank)
gen.X <- function(dist.mean, dist.rank, nl, n.of.points, q, k, xi) {

  X <- array(dim=c(n.of.points,dim(dist.mean)[2]))
  idx <- sample(dim(dist.mean)[1],size=n.of.points,
    replace=TRUE,prob=dnorm(dist.rank,1,q*k))

  # iterate through the chosen distributions
  for (l in 1:length(idx)) {
    j <- idx[l]
    # rotate the coordinate system
    o.dist.mean <- t(t(dist.mean) - dist.mean[j,])  # transaltion of origin
    r.dist.mean <- o.dist.mean
    set <- nl[j,]  # set of available neighbours
    vec <- vector()
    for (m in 1:(dim(dist.mean)[2]-1)) {
      dis <- apply(matrix(r.dist.mean[set,m:dim(r.dist.mean)[2]],
        length(set),length(m:dim(r.dist.mean)[2])),1,euc.dist)
      if (sum(dis)==0.0)  return(NULL) # if the distribution have converged
      if (length(set)>1)
        choice <- sample(set,size=1,prob=dis^4)
      else
        choice <- set
      vec <- cbind(vec,o.dist.mean[choice,])
      R <- qr.Q(qr(vec), complete=TRUE) # rot. matrix after orthogonalization   
      if (det(R)<0) {
        R[,1] <- -R[,1]
      }
      r.dist.mean <-  o.dist.mean %*% R # rotated coordinates
      set <- set[set!=choice]
    }

    dist.sd <- vector()
    for (i in 1:dim(dist.mean)[2]) {
      dist.sd <- c(dist.sd,sum(abs(r.dist.mean[nl[j,],i]-
        r.dist.mean[j,i]))/(k-1))
    }
    n.x <- rnorm(dim(dist.mean)[2],r.dist.mean[j,],dist.sd*xi)
    n.x <- R %*% n.x
    n.x <- t(n.x + dist.mean[j,])
    X[l,] <- n.x
  }
  return(X)
}

# Generates a set of values for discrete variables based on the solution 
# archive (with probability of using a distribution based on its rank)
gen.C <- function(p.C, dist.rank, range.C, n.of.points, q, k, xi) {

  C <- array(dim=c(n.of.points,dim(p.C)[2]))
  # iterate through all the (u) dimiensions
  for (i in 1:(dim(p.C)[2])) {
    best.q.rank <- vector()
    total <- vector()
    # iterate through all possible values
    for (j in 1:(range.C[i])) {
      idx <- p.C[,i]==j
      total[j] <- sum(idx)
      if (total[j]>0) 
        best.q.rank[j] <- min(dist.rank[idx])
      else {
        best.q.rank[j] <- Inf
        total[j] <- -1 # to avoid symbol Inf*0
      }
    }         
    prob.dist <- dnorm(best.q.rank,1,q*k)/total
    prob.dist[prob.dist<0] <- 0
    prob.dist <- prob.dist/max(prob.dist)
    if (sum(prob.dist==0)>0)
      prob.dist <- prob.dist + q/sum(prob.dist==0)
    C[,i] <- sample(range.C[i],size=n.of.points,replace=TRUE,prob=prob.dist)
  } 

  return(C)  
}
