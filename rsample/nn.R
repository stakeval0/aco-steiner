#######################################################################
#
# nn.R - R implementation of functions related to feed-forward neural 
#         networks - a function for evaluating the output of the neural
#         network based on a given input, and two network training 
#         algorithms: backpropagation and Levenberg-Marquardt.
#
# Copyright (c) 2006 by Krzysztof Socha (ksocha@ulb.ac.be)
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by 
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#

# Evaluates the NN (feed forward MLP, one hidden layer)
#  s=c(n.i,n.h,n.o)
#		n.i - number of input nodes
#		n.h - number of hidden nodes
#		n.o - number of output nodes 
#	 w=c(w.h.1,..,w.h.(n.i+1)*n.h,w.o.1,..,w.o.(n.h+1)*n.o)
#		w.h - weights of the hidden nodes 
#		w.o - weights of the output nodes
#  input - the input vector

eval.NN <- function(s,w,input) {

	# check if the parameters make sense
	if ((s[1]+1)*s[2]+(s[2]+1)*s[3]!=length(w) | s[1]!=length(input))
		stop("The struture of NN does not match the weight list and/or the input length.")

	# initialize and parse weight matrices
	b <- list(vector(),vector()) # bias
	wm <- list(
		matrix(w[1:((s[1]+1)*s[2])],s[1]+1,s[2]),
		matrix(w[((s[1]+1)*s[2]+1):length(w)],s[2]+1,s[3]))
	b[[1]] <- wm[[1]][dim(wm[[1]])[1],]
	b[[2]] <- wm[[2]][dim(wm[[2]])[1],]
	wm[[1]] <- wm[[1]][1:(dim(wm[[1]])[1]-1),]
	wm[[2]] <- wm[[2]][1:(dim(wm[[2]])[1]-1),]
	in.signal <- list(input,vector())
	out.signal <- list(vector(),vector())

	# perform the evalution
	for (l in 1:2) {
		for (i in 1:s[l+1])
			out.signal[[l]] <- c(out.signal[[l]],sigmoid(sum(in.signal[[l]]*wm[[l]][,i]),b[[l]][i]))
		if (l<2)
			in.signal[[l+1]] <- out.signal[[l]]
	}

	# return the output
	return(list(out.signal,in.signal))
}

# returns the value of the sigmoid function
sigmoid <- function(x,b) {
	return(1/(1+exp(1)^-(x+b)))
}

# Backpropagetes the NN (feed forward MLP, one hidden layer)
#  s=c(n.i,n.h,n.o)
#		n.i - number of input nodes
#		n.h - number of hidden nodes
#		n.o - number of output nodes 
#	 w=c(w.h.1,..,w.h.(n.i+1)*n.h,w.o.1,..,w.o.(n.h+1)*n.o)
#		w.h - weights of the hidden nodes 
#		w.o - weights of the output nodes
#  input - the input vector
#  output - the expected output vector
#  eta - learning rate

bp.NN <- function(s,w,input,output,eta) {

	delta <- list(vector(),vector())

	# initialize and parse weight matrices
	b <- list(vector(),vector())
	wm <- list(
		matrix(w[1:((s[1]+1)*s[2])],s[1]+1,s[2]),
		matrix(w[((s[1]+1)*s[2]+1):length(w)],s[2]+1,s[3]))
	b[[1]] <- wm[[1]][dim(wm[[1]])[1],]
	b[[2]] <- wm[[2]][dim(wm[[2]])[1],]
	wm[[1]] <- wm[[1]][1:(dim(wm[[1]])[1]-1),]
	wm[[2]] <- wm[[2]][1:(dim(wm[[2]])[1]-1),]
	
	# initialize and parse NN values
	signal <- eval.NN(s,w,input)
	out.signal <- signal[[1]]
	in.signal <- signal[[2]]

	# calculate the delta for the output layer
	delta[[2]] <- (output-out.signal[[2]])*out.signal[[2]]*(1-out.signal[[2]])
	# calculate the delta for the hidden layer
	delta[[1]] <- out.signal[[1]]*(1-out.signal[[1]])*apply(t(delta[[2]]*t(wm[[2]])),2,sum)

	# update the weight and bias matrices
	wm[[2]] <- wm[[2]] + eta * t(delta[[2]] %*% t(in.signal[[2]]))
	wm[[1]] <- wm[[1]] + eta * t(delta[[1]] %*% t(in.signal[[1]]))
	wm[[2]] <- rbind(wm[[2]], b[[2]] + eta * delta[[2]])
	wm[[1]] <- rbind(wm[[1]], b[[1]] + eta * delta[[1]])
	
	return(c(as.vector(wm[[1]]),as.vector(wm[[2]])))
}


# Levenberg-Marquardt algorithm for trainign the NN (feed forward MLP, one hidden layer)
#  s=c(n.i,n.h,n.o)
#		n.i - number of input nodes
#		n.h - number of hidden nodes
#		n.o - number of output nodes 
#	 w=c(w.h.1,..,w.h.(n.i+1)*n.h,w.o.1,..,w.o.(n.h+1)*n.o)
#		w.h - weights of the hidden nodes 
#		w.o - weights of the output nodes
#  input - the input matrix (all the training patterns)
#  output - the expected output matrix
#  beta - learning rate
#  miu - initial value of the cooeficient
#  epochs - number of epochs to perform
#  f - function calculating the error: f(w)

lm.NN <- function(s,w,input,output,miu,beta,epochs,f,a,b) {

	# calculate the initial error
	e1 <- abs(f(w,a,b))
	#e2 <- e1

	# iterate the required number of epochs
	epoch <- 0
	while (epoch<epochs) {

		# create the Jacobian matrix
		J <- matrix(0,s[3]*dim(input)[1],length(w))
		# create the error matrix
		e <- matrix(0,1,s[3]*dim(input)[1])

		# initialize and parse weight matrices
		bias <- list(vector(),vector())
		wm <- list(
			matrix(w[1:((s[1]+1)*s[2])],s[1]+1,s[2]),
			matrix(w[((s[1]+1)*s[2]+1):length(w)],s[2]+1,s[3]))
		bias[[1]] <- wm[[1]][dim(wm[[1]])[1],]
		bias[[2]] <- wm[[2]][dim(wm[[2]])[1],]
		wm[[1]] <- wm[[1]][1:(dim(wm[[1]])[1]-1),]
		wm[[2]] <- wm[[2]][1:(dim(wm[[2]])[1]-1),]

		# iterate through all the training patterns
		for (p in 1:(dim(input)[1])) {
		 
			# initialize and parse NN values
			signal <- eval.NN(s,w,as.numeric(input[p,]))
			out.signal <- signal[[1]]
			in.signal <- signal[[2]]

			# calculate the output part of the Jacobian matrix
			# weights
			for (k in 1:s[3])
				for (j in 1:s[2]) 
					J[k+k*(p-1),(s[1]+1)*s[2]+j+(s[2]+1)*(k-1)] <- - out.signal[[1]][j] * out.signal[[2]][k] * (1 - out.signal[[2]][k])
			# bias
			for (k in 1:s[3])
				J[k+k*(p-1),(s[1]+1)*s[2]+(s[2]+1)*k] <- - out.signal[[2]][k] * (1 - out.signal[[2]][k])

			# calculate the hidden part of the Jacobian matrix
			# weights
			for (k in 1:s[3])
				for (j in 1:s[2]) 
					for (i in 1:s[1])
						J[k+k*(p-1),i+(s[1]+1)*(j-1)] <- - in.signal[[1]][i] * out.signal[[2]][k] * (1 - out.signal[[2]][k]) * wm[[2]][j,k] * out.signal[[1]][j] * (1 - out.signal[[1]][j])
			# bias
			for (k in 1:s[3])
				for (j in 1:s[2]) 
					J[k+k*(p-1),(s[1]+1)*j] <- - out.signal[[1]][j] * (1 - out.signal[[1]][j])
			
			# calculate the error matrix
			for (k in 1:s[3])
				e[1,k+k*(p-1)] <- output[p,k] - out.signal[[2]][k]
		}

		# calculate the improvement		
		e2 <- Inf
		while (e2>e1) {
			d.w <- e %*% J %*% solve(t(J)%*%J + miu*diag(1,dim(J)[2]))
			e2 <- abs(f(w-as.numeric(d.w),a,b))
			if (e1<e2) miu <- miu * beta
			epoch <- epoch + 1
		}
		miu <- miu / beta
		w <- w - as.numeric(d.w)
		e1 <- e2
		#print(e2,quote=FALSE)
	}
	#print(paste("Final error: ",f(w)),quote=FALSE)			
	assign("evals.done",epoch-1,.GlobalEnv)
	# return the updated vector
	return(w)
}