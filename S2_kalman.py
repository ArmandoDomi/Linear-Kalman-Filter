# -*- coding: utf-8 -*-
"""
Created on Wed May 20 21:57:51 2020

@author: Armando
"""


import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from filterpy.stats import plot_covariance_ellipse
from time import sleep
from mpl_toolkits import mplot3d

class Kalman:
    
    def __init__(self,X,Q,R,P):
        self.X=X
        self.Q=Q
        self.R=R
        self.P=P
        
        
    
    def update(self,Z,H):
        
        Y = Z - np.matmul(H.T,self.X)
        S = np.matmul(H.T,np.matmul(self.P,H))+self.R
        temp1=np.matmul(H,1/S)
        #np.matmul(H.T,np.linalg.inv(S))
        K = np.matmul(self.P,temp1)

        self.X= self.X+ np.matmul(K,Y)
        #self.X= self.X+ np.matmul(K.T,Y)
        temp= np.matmul(K,H.T)
        self.P=np.matmul((np.eye(temp.shape[0])-temp),self.P)
        return (self.X,self.P,K,Y)
        
        
        
    def predict(self,F,B,U):
        
        self.X = np.matmul(F,self.X)+np.matmul(B,U)
        self.P = np.matmul(F,np.matmul(self.P,F.T))+self.Q
        return (self.X,self.P)
        
    
    
    
    
    
    
if __name__ == "__main__":
    """ Executed only when the file is run as a script. """
    

    #np.random.seed(123)
    # Initialization of state matrices
    
    X0 = np.array([[0],[0]])
    X = np.array([[0],[0]])             # initial state E(X(0))=0
    P = np.diag((40,40))                # initial state covariance
    F = np.array([[0.8,2], [0,0.9]])    # the state-transition model
    Q = np.diag((0,0))                # the covariance of the process noise
    B = np.diag((1,1))                  # the control-input model
    U = np.array([[3],[5]])             # control vector

    

    Ek=np.array([0,0])                  #Controll input model
      
    
    G=np.diag((1,1))
    #G=np.array([[1],[1]])
    
    # Measurement matrices    
    H = np.array([[1], [1]])                #the observation model
    #R = np.diag((6,6))                     #the covariance of the observation noise
    R=6
    
    # number of iteration
    N_iter=100
    
    # outputs of Kalman update
    outputX=[]
    outputP=[]
    outputK=[]
    outputY_hat=[]
    
    #Kalman Predict
    
    X_pre=[]
    P_pre=[]
    
    
    #fig = plt.figure()
    #ax1 = fig.add_subplot(111)
    
    #init Kalman Filter
    kalman_filter = Kalman(X,Q,R,P)

    outputX.append(X)
    outputP.append(P)
    
    # lists for the motion model Xk,Y
    ListXk=[]
    ListY=[]
    
    
    X_pures=[]
    
    Xk=X
    Xpepe=X
    
    ListXk.append(X)
    X_pures.append(X)
    
    fig = plt.figure()
    ax10 = fig.add_subplot(111)
    plot_covariance_ellipse([0,0],P, facecolor='lightskyblue', alpha=0.2)


    # applying kalman filter
    for t in range (0,N_iter):
        
        w1=np.random.normal(0, np.sqrt(0), 1)
        w2=np.random.normal(0, np.sqrt(0), 1)
        
        # process noise
        W =np.array([[w1[0]],[w2[0]]])
    
        e=np.random.normal(0, np.sqrt(R), 1)
        
        # motion model
        Xk=np.matmul(F,Xk)+np.matmul(B,U)+np.matmul(G,W)
        
        #motion model with no noise
        Xpepe=np.matmul(F,Xpepe)+np.matmul(B,U)
        X_pures.append(Xpepe)
               
        ListXk.append(Xk)
        
        # measurement model
        Y=np.matmul(H.T,Xk)+np.matmul(Ek,U)+e
        #Y=np.matmul(np.diag((1,1)),Xk)+np.matmul(Ek,U)+e
        
        ListY.append(Y)
        
        
        # Kalman predict
        (X,P)=kalman_filter.predict(F,B,U)
        
        X_pre.append(X)
        P_pre.append(P)
        
        
                
        #Kalman update
        (X,P,K,Y_hat)=kalman_filter.update(Y,H)
        
        

        outputX.append(kalman_filter.X)
        outputP.append(kalman_filter.P)
        outputK.append(K)
        outputY_hat.append(Y_hat)
        
        if t <= 20:
            plot_covariance_ellipse(X, P, facecolor='lightskyblue', alpha=0.2)
            ax10.set_title('Kalman filter iter=%d'%(t))
            plt.xlabel('X1')
            plt.ylabel('X2')
            plt.legend(['Uncertainty of Prediction'])
            ax10.relim()
            ax10.autoscale_view()
    
            sleep(0.05)
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            fig.savefig('images_s2/'+str(1)+'.png')
        
        
        
        
    #Xk
    list_x1=[]
    list_x2=[]
    #outputX
    list_x1_hat=[]
    list_x2_hat=[]
    #Kalman Gain
    gain=[]
    
    
    x1_pre=[]
    x2_pre=[]
    
    errorx1=[]
    errorx2=[]
    
    P1=[]
    P2=[]
    P3=[]
    P4=[]
    
    
    # arrays for the pure model
    x1_pure=[]
    x2_pure=[]
    
    #arrays for kalman gain
    K1=[]
    K2=[]
    
    for i in range(N_iter):
        
        list_x1.append(ListXk[i][0][0])
        list_x2.append(ListXk[i][1][0])
        
        # lists for x1,x2 without noise
        x1_pure.append(X_pures[i][0][0])
        x2_pure.append(X_pures[i][1][0])
                    
    
        list_x1_hat.append(outputX[i][0][0])
        list_x2_hat.append(outputX[i][1][0])
        
        
        
        x1_pre.append(X_pre[i][0][0])
        x2_pre.append(X_pre[i][1][0])
        
        
        gain.append(outputK[i][0])
        
        #errors
        errorx1.append(abs(list_x1[i]-list_x1_hat[i]))
        errorx2.append(abs(list_x2[i]-list_x2_hat[i]))
        
        # Ps
        P1.append(outputP[i][0][0]) #k
        P2.append(outputP[i][0][1])
        P3.append(outputP[i][1][0])
        P4.append(outputP[i][1][1]) #k
        
        # kalman gain
        K1.append(outputK[i][0])
        K2.append(outputK[i][1])
                
        
        
    
    N=[i for i in range(0,50)]
    

     

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
                
    plt.plot(list_x1,'-r')
    plt.plot(list_x1_hat,'-b')
    #plt.plot(errorx1,'-g')
    plt.figtext(0.6,0.9,'Q ='+str(0)+', R = '+str(R)+', X ='+str([0,0]))
    plt.legend(['X1','X1_hat'])
    plt.xlabel('Iterations')
    plt.ylabel('X1')
    fig.savefig('images_s2/'+str(2)+'.png')
                            
    #plot x2
    fig = plt.figure()
    ax2 = fig.add_subplot(111)
                
    plt.plot(list_x2,'-r')
    plt.plot(list_x2_hat,'-b')
    #plt.plot(errorx2,'-g')
    plt.figtext(0.6,0.9,'Q ='+str(0)+', R = '+str(R)+', X ='+str([0,0]))
    plt.legend(['X2','X2_hat'])
    plt.xlabel('Iterations')
    plt.ylabel('X2')
    fig.savefig('images_s2/'+str(3)+'.png')
    
    
    #P plots
    fig = plt.figure()
    ax3 = fig.add_subplot(111)
                
    plt.plot(P1,'-r')
    plt.plot(P2,'-b')
    plt.plot(P3,'-g')
    plt.plot(P4,'-y')
    plt.legend(['P1','P2','P3','P4'])
    plt.xlabel('Iterations')
    plt.ylabel('P')
    plt.figtext(0.6,0.9,'Q ='+str(0)+', R = '+str(R)+', X ='+str([0,0]))
    fig.savefig('images_s2/'+str(4)+'.png')
    
    
    
   
    #errorX2 plot
    fig = plt.figure()
    ax4 = fig.add_subplot(111)
                
    plt.plot(errorx1,'-g')
    #plt.legend(['Estimated error of X1'])
    plt.figtext(0.6,0.9,'Q ='+str(0)+', R = '+str(R)+', X ='+str([0,0]))
    plt.xlabel('Iterations')
    plt.ylabel('X1 error')
    fig.savefig('images_s2/'+str(5)+'.png')
                
   
    #errorX1 plot
    fig = plt.figure()
    ax5 = fig.add_subplot(111)
    
    plt.plot(errorx2,'-g')
    #plt.legend(['Estimated error of X2'])
    plt.figtext(0.6,0.9,'Q ='+str(0)+', R = '+str(R)+', X ='+str([0,0]))
    plt.xlabel('Iterations')
    plt.ylabel('X2 error')
    fig.savefig('images_s2/'+str(6)+'.png')
    
    '''
    
    fig = plt.figure()
    ax6 = fig.add_subplot(111)
    
    
    plt.plot(K1,'-r')
    plt.plot(K2,'-b')
    plt.legend(['Kalman Gain K1','Kalman Gain K2'])
    plt.figtext(0.6,0.9,'Q ='+str(10)+', R = '+str(6)+', X ='+str([0,0]))
    plt.xlabel('Iteration')
    plt.ylabel('K')
    fig.savefig('images__s1/'+str(7)+'.png')
    '''    
                

    
    

    
        
    