#!/usr/bin/python3
import numpy as np
import math

def Smtrx(a):
    """
    S = Smtrx(a) computes the 3x3 vector skew-symmetric matrix S(a) = -S(a)'.
    The cross product satisfies: a x b = S(a)b. 
    """
 
    S = np.array([ 
        [ 0, -a[2], a[1] ],
        [ a[2],   0,     -a[0] ],
        [-a[1],   a[0],   0 ]  ])

    return S

def Hmtrx(r):
    """
    H = Hmtrx(r) computes the 6x6 system transformation matrix
    H = [eye(3)     S'
         zeros(3,3) eye(3) ]       Property: inv(H(r)) = H(-r)

    If r = r_bg is the vector from the CO to the CG, the model matrices in CO and
    CG are related by: M_CO = H(r_bg)' * M_CG * H(r_bg). Generalized position and
    force satisfy: eta_CO = H(r_bg)' * eta_CG and tau_CO = H(r_bg)' * tau_CG 
    """

    H = np.identity(6,float)
    H[0:3, 3:6] = Smtrx(r).T

    return H


def m2c(M, nu):
    """
    C = m2c(M,nu) computes the Coriolis and centripetal matrix C from the
    mass matrix M and generalized velocity vector nu (Fossen 2021, Ch. 3)
    """

    M = 0.5 * (M + M.T)     # systematization of the inertia matrix

    if (len(nu) == 6):      #  6-DOF model
    
        M11 = M[0:3,0:3]
        M12 = M[0:3,3:6] 
        M21 = M12.T
        M22 = M[3:6,3:6] 
    
        nu1 = nu[0:3]
        nu2 = nu[3:6]
        dt_dnu1 = np.matmul(M11,nu1) + np.matmul(M12,nu2)
        dt_dnu2 = np.matmul(M21,nu1) + np.matmul(M22,nu2)

        #C  = [  zeros(3,3)      -Smtrx(dt_dnu1)
        #      -Smtrx(dt_dnu1)  -Smtrx(dt_dnu2) ]
        C = np.zeros( (6,6) )    
        C[0:3,3:6] = -Smtrx(dt_dnu1)
        C[3:6,0:3] = -Smtrx(dt_dnu1)
        C[3:6,3:6] = -Smtrx(dt_dnu2)
            
    else:   # 3-DOF model (surge, sway and yaw)
        #C = [ 0             0            -M(2,2)*nu(2)-M(2,3)*nu(3)
        #      0             0             M(1,1)*nu(1)
        #      M(2,2)*nu(2)+M(2,3)*nu(3)  -M(1,1)*nu(1)          0  ]    
        C = np.zeros( (3,3) ) 
        C[0,2] = -M[1,1] * nu[1] - M[1,2] * nu[2]
        C[1,2] =  M[0,0] * nu[0] 
        C[2,0] = -C[0,2]       
        C[2,1] = -C[1,2]
        
    return C

def Rzyx(phi,theta,psi):
    """
    R = Rzyx(phi,theta,psi) computes the Euler angle rotation matrix R in SO(3)
    using the zyx convention
    """
    
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    cth  = math.cos(theta)
    sth  = math.sin(theta)
    cpsi = math.cos(psi)
    spsi = math.sin(psi)
    
    R = np.array([
        [ cpsi*cth, -spsi*cphi+cpsi*sth*sphi, spsi*sphi+cpsi*cphi*sth ],
        [ spsi*cth,  cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi ],
        [ -sth,      cth*sphi,                 cth*cphi ] ])

    return R


def sat(x, x_min, x_max):
    """
    x = sat(x,x_min,x_max) saturates a signal x such that x_min <= x <= x_max
    """
    if x > x_max:
        x = x_max 
    elif x < x_min:
        x = x_min
        
    return x  


def crossFlowDrag(L,B,T,nu_r):
    """
    tau_crossflow = crossFlowDrag(L,B,T,nu_r) computes the cross-flow drag 
    integrals for a marine craft using strip theory. 

    M d/dt nu_r + C(nu_r)*nu_r + D*nu_r + g(eta) = tau + tau_crossflow
    """

    rho = 1026               # density of water
    n = 20                   # number of strips

    dx = L/20             
    Cd_2D = Hoerner(B,T)    # 2D drag coefficient based on Hoerner's curve

    Yh = 0
    Nh = 0
    xL = -L/2
    
    for i in range(0,n+1):
        v_r = nu_r[1]             # relative sway velocity
        r = nu_r[5]               # yaw rate
        Ucf = abs(v_r + xL * r) * (v_r + xL * r)
        Yh = Yh - 0.5 * rho * T * Cd_2D * Ucf * dx         # sway force
        Nh = Nh - 0.5 * rho * T * Cd_2D * xL * Ucf * dx    # yaw moment
        xL += dx
        
    tau_crossflow = np.array([0, Yh, 0, 0, 0, Nh],float)

    return tau_crossflow


def Hoerner(B,T):
    """
    CY_2D = Hoerner(B,T)
    Hoerner computes the 2D Hoerner cross-flow form coeff. as a function of beam 
    B and draft T.The data is digitized and interpolation is used to compute 
    other data point than those in the table
    """
    
    # DATA = [B/2T  C_D]
    DATA1 = np.array([
        0.0109,0.1766,0.3530,0.4519,0.4728,0.4929,0.4933,0.5585,0.6464,0.8336,
        0.9880,1.3081,1.6392,1.8600,2.3129,2.6000,3.0088,3.4508, 3.7379,4.0031 
        ])
    DATA2 = np.array([
        1.9661,1.9657,1.8976,1.7872,1.5837,1.2786,1.2108,1.0836,0.9986,0.8796,
        0.8284,0.7599,0.6914,0.6571,0.6307,0.5962,0.5868,0.5859,0.5599,0.5593 
        ])

    CY_2D = np.interp( B / (2 * T), DATA1, DATA2 )
        
    return CY_2D

def attitudeEuler(eta,nu,sampleTime):
    """
    eta = attitudeEuler(eta,nu,sampleTime) computes the generalized 
    position/Euler angles eta[k+1]
    """
   
    p_dot   = np.matmul( Rzyx(eta[3], eta[4], eta[5]), nu[0:3] )
    v_dot   = np.matmul( Tzyx(eta[3], eta[4]), nu[3:6] )

    # Forward Euler integration
    eta[0:3] = eta[0:3] + sampleTime * p_dot
    eta[3:6] = eta[3:6] + sampleTime * v_dot

    return eta


def Tzyx(phi,theta):
    """
    T = Tzyx(phi,theta) computes the Euler angle attitude
    transformation matrix T using the zyx convention
    """
    
    cphi = math.cos(phi)
    sphi = math.sin(phi)
    cth  = math.cos(theta)
    sth  = math.sin(theta)    

    try: 
        T = np.array([
            [ 1,  sphi*sth/cth,  cphi*sth/cth ],
            [ 0,  cphi,          -sphi],
            [ 0,  sphi/cth,      cphi/cth] ])
        
    except ZeroDivisionError:  
        print ("Tzyx is singular for theta = +-90 degrees." )
        
    return T


def quaternion_from_euler(roll, pitch, yaw):
    """
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw