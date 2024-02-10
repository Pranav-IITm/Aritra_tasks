#!/usr/bin/python3
import numpy as np
from aritra_task2.utils import quaternion_from_euler
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from std_msgs.msg import Float32MultiArray

from scipy.integrate import solve_ivp

class KCS_Vessel:
    def __init__(self, vessel_params) -> None:

        self.g = 9.80665
        self.rho = 1025

        # Ship Geometry and Constants

        self.vessel_params = vessel_params

    def ode(self, t, v, world_params, actions):
        """
        ode to simulate Own ship to perform waypoint tracking.
        Currently set to a control propeller input
        later can be updated with a speed control
        """
        # Nondimensional State Space Variables
        up = v[0]
        vp = v[1]
        rp = v[2]
        xp = v[3]
        yp = v[4]
        psi = v[5]
        delta = v[6]
        n_prop = v[7]

        L = self.vessel_params['L']
        B = self.vessel_params['B']
        d_em = self.vessel_params['d_em']
        Cb = self.vessel_params['Cb']
        Fn = self.vessel_params['Fn']

        wind_flag = world_params['wind_flag']
        wind_speed = world_params['wind_speed']
        wind_dir = world_params['wind_dir']

        wave_flag = world_params['wave_flag']
        wave_height = world_params['wave_height']
        wave_period = world_params['wave_period']
        wave_dir = world_params['wave_dir']

        delta_c, n_c = actions

        # n_prop = 115.5/60 # change this later (to account for speed control)

        # Derived kinematic variables
        b = np.arctan2(-vp, up)     # Drift angle

        # Ship Geometry and Constants
        rho = 1025
        g = 9.80665

        Dsp = Cb * L * B * d_em
        U_des = Fn * np.sqrt(g * L)
        xG = -3.404
        kzzp = 0.25

        # Surge Hydrodynamic Derivatives in non-dimensional form
        X0 = -0.0167
        Xbb = -0.0549
        Xbr_minus_my = -0.1084
        Xrr = -0.0120
        Xbbbb = -0.0417

        # Sway Hydrodynamic Derivatives in non-dimensional form
        Yb = 0.2252
        Yr_minus_mx = 0.0398
        Ybbb = 1.7179
        Ybbr = -0.4832
        Ybrr = 0.8341
        Yrrr = -0.0050

        # Yaw Hydrodynamic Derivatives in non-dimensional form
        Nb = 0.1111
        Nr = -0.0465
        Nbbb = 0.1752
        Nbbr = -0.6168
        Nbrr = 0.0512
        Nrrr = -0.0387

        # Non-dimensional Surge Hull Hydrodynamic Force
        Xp_H = X0 * (up ** 2) \
            + Xbb * (b ** 2) + Xbr_minus_my * b * rp \
            + Xrr * (rp ** 2) + Xbbbb * (b ** 4)

        # Non-dimensional Sway Hull Hydrodynamic Force
        Yp_H = Yb * b + Yr_minus_mx * rp + Ybbb * (b ** 3) \
            + Ybbr * (b ** 2) * rp + Ybrr * b * (rp ** 2) \
            + Yrrr * (rp ** 3)

        # Non-dimensional Yaw Hull Hydrodynamic Moment
        Np_H = Nb * b + Nr * rp + Nbbb * (b ** 3) \
            + Nbbr * (b ** 2) * rp + Nbrr * b * (rp ** 2) \
            + Nrrr * (rp ** 3)

        # Propulsion Force Calculation

        # The value self propulsion RPM is taken from Yoshimura's SIMMAN study
        # Analysis of steady hydrodynamic force components and prediction of
        # manoeuvering ship motion with KVLCC1, KVLCC2 and KCS
        # n_prop = 115.5 / 60
        Dp = 7.9
        wp = 1 - 0.645  # Effective Wake Fraction of the Propeller
        tp = 1 - 0.793  # Thrust Deduction Factor

        J = (up * U_des) * (1 - wp) / (n_prop * Dp)     # Advance Coefficient

        a0 = 0.5228
        a1 = -0.4390
        a2 = -0.0609
        Kt = a0 + a1 * J + a2 * (J ** 2)        # Thrust Coefficient

        # Dimensional Propulsion Force
        X_P = (1 - tp) * rho * Kt * (Dp ** 4) * (n_prop ** 2)

        # Non-dimensional Propulsion Force
        Xp_P = X_P / (0.5 * rho * L * d_em * (U_des ** 2))

        # Rudder Force Calculation
        A_R = L * d_em / 54.86
        Lamda = 2.164
        f_alp = 6.13 * Lamda / (2.25 + Lamda)

        eps = 0.956
        eta = 0.7979
        kappa = 0.633
        # Assuming propeller location is 10 m ahead of AP (Rudder Location)
        xp_P = -0.4565
        xp_R = -0.5

        b_p = b - xp_P * rp

        if b_p > 0:
            gamma_R = 0.492
        else:
            gamma_R = 0.338

        lp_R = -0.755

        up_R = eps * (1 - wp) * up * np.sqrt(eta * (1 + kappa *
                                                    (np.sqrt(1 + 8 * Kt / (np.pi * (J ** 2))) - 1)) ** 2 + (1 - eta))

        vp_R = gamma_R * (vp + rp * lp_R)

        Up_R = np.sqrt(up_R ** 2 + vp_R ** 2)
        alpha_R = delta - np.arctan2(-vp_R, up_R)

        F_N = A_R / (L * d_em) * f_alp * (Up_R ** 2) * np.sin(alpha_R)

        tR = 1 - 0.742
        aH = 0.361
        xp_H = -0.436

        Xp_R = - (1 - tR) * F_N * np.sin(delta)
        Yp_R = - (1 + aH) * F_N * np.cos(delta)
        Np_R = - (xp_R + aH * xp_H) * F_N * np.cos(delta)

        # Coriolis terms

        mp = Dsp / (0.5 * (L ** 2) * d_em)
        xGp = xG / L

        Xp_C = mp * vp * rp + mp * xGp * (rp ** 2)
        Yp_C = -mp * up * rp
        Np_C = -mp * xGp * up * rp

        # ----------------------------------------------------
        # Wind Force Calculation
        # ----------------------------------------------------

        if wind_flag == 1:
            Vw = wind_speed  # wind speed
            betaw = wind_dir * (np.pi/180)  # wind direction
            Lp = 3.0464
            de = 0.1430
            uw = Vw * np.cos(betaw - psi)
            vw = Vw * np.sin(betaw - psi)
            urw = up - uw
            vrw = vp - vw
            Uwr = (urw ** 2 + vrw ** 2) ** 0.5
            gammaw = np.arctan2(-vrw, -urw)
            # print(gammaw,"gamma")

            rhow = 1025
            rhoa = 1.225
            Ax = (0.4265 * (0.2517 - 0.1430))
            Ay = ((0.2517 - 0.1430) * Lp)
            # print(Ax,Ay,"AX")

            Cwx = 1 * np.cos(gammaw)
            Cwy = 1 * np.sin(gammaw)
            Cwpsi = 0.5 * np.sin(gammaw)

            Xp_W = (Ax * Cwx * Uwr * abs(Uwr)) * rhoa / (Lp * de * rhow)
            Yp_W = (Ay * Cwy * Uwr * abs(Uwr)) * rhoa / (Lp * de * rhow)
            Np_W = (Ay * Lp * Cwpsi * Uwr * abs(Uwr)) * \
                rhoa / ((Lp ** 2) * de * rhow)
        else:
            Xp_W = 0.0
            Yp_W = 0.0
            Np_W = 0.0

        # Net non-dimensional force and moment computation
        Xp = Xp_H + Xp_R + Xp_C + Xp_P + Xp_W
        Yp = Yp_H + Yp_R + Yp_C + Yp_W
        Np = Np_H + Np_R + Np_C + Np_W

        # Net force vector computation
        X = Xp
        Y = Yp
        N = Np

        # Added Mass and Mass Moment of Inertia (from MDLHydroD)
        mxp = 1790.85 / (0.5 * (L ** 2) * d_em)
        myp = 44324.18 / (0.5 * (L ** 2) * d_em)
        Jzzp = 140067300 / (0.5 * (L ** 4) * d_em)
        Izzp = mp * (kzzp ** 2) + mp * (xGp ** 2)

        Mmat = np.zeros((3, 3))

        Mmat[0, 0] = mp + mxp
        Mmat[1, 1] = mp + myp
        Mmat[2, 2] = Izzp + Jzzp
        Mmat[1, 2] = mp * xGp
        Mmat[2, 1] = mp * xGp

        Mmatinv = np.linalg.inv(Mmat)

        tau = np.array([X, Y, N])

        vel_der = Mmatinv @ tau

        # Derivative of state vector
        vd = np.zeros_like(v)

        vd[0:3] = vel_der
        vd[3] = up * np.cos(psi) - vp * np.sin(psi)
        vd[4] = up * np.sin(psi) + vp * np.cos(psi)
        vd[5] = rp

        T_rud = 1     # Corresponds to a time constant of 1 * L / U_des = 20 seconds
        deltad = (delta_c - delta) / T_rud
        # Maximum rudder rate of 5 degrees per second
        deltad_max = 5 * np.pi / 180 * (L / U_des)

        # Rudder rate saturation
        if np.abs(deltad) > deltad_max:
            deltad = np.sign(deltad) * deltad_max

        T_prop = 1     # Corresponds to a time constant of 1 * L / U_des = 20 seconds
        nd_prop = (n_c - n_prop) / T_prop
        nd_max = 0.1

        # Propeller RPM rate saturation
        if np.abs(nd_prop) > nd_max:
            nd_prop = np.sign(nd_prop) * nd_max

        vd[6] = deltad
        # currently set to zero (later need to be changed for considering the speed control)
        vd[7] = nd_prop
        return vd



class KCS(Node):
    def __init__(self, vessel_params, world_params):
        super().__init__('kcs')

        self.get_logger().info("KCS INIT")

        self.kcs = KCS_Vessel(vessel_params)
        self.world_params = world_params
        self.loop = self.create_timer(timer_period_sec=0.1, callback=self.step)
        self.state = np.ones(8, float) * 1e-5

        self.n_prop = float(115.5/60)
        self.delta_c = float(0)

        self.state_pub = self.create_publisher(Odometry, 'kcs/odom', 10)
        self.path_pub = self.create_publisher(Path, 'kcs/path', 10)

        self.action_sub = self.create_subscription(Float32MultiArray, 'kcs/commands', self.action_callback, 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def action_callback(self, msg: Float32MultiArray):
        delta_c, n_prop = msg.data.tolist()

        self.delta_c = max(-np.radians(35), min(np.radians(35), delta_c))

        self.n_prop = min(115.5/30 , n_prop)

        # self.get_logger().info(f'Rudder Angle :{self.delta_c}')
        # self.get_logger().info(f'Prop RPM :{self.n_prop}')

    def step(self, world_params = None):
        tspan = (0, 0.1)
        yinit = self.state
        actions = [self.delta_c, self.n_prop]

        if world_params is not None:
           self.world_params = world_params

        sol = solve_ivp(lambda t,v: self.kcs.ode(t, v, self.world_params, actions=actions),
                        tspan, yinit, t_eval=tspan, dense_output=True)
        
        psi_rad = sol.y[5][-1]  # psi
        psi = (psi_rad + np.pi) % (2 * np.pi) - np.pi

        self.state[0] = sol.y[0][-1]    # Surge velocity
        self.state[1] = sol.y[1][-1]    # Sway velocity
        self.state[2] = sol.y[2][-1]    # Yaw velocity
        self.state[3] = sol.y[3][-1]    # X cooridnate
        self.state[4] = sol.y[4][-1]    # Y coordinate
        self.state[5] = psi             # Heading angle
        self.state[6] = sol.y[6][-1]    # Actual rudder angle
        self.state[7] = sol.y[7][-1]

        u, v, r, x, y, psi, delta_c, n_prop = self.state

        stamp = self.get_clock().now().to_msg()
        pose_msg = Pose()
        pose_msg.position.x,pose_msg.position.y = x,y

        (pose_msg.orientation.w,
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z) = quaternion_from_euler(0 , 0, psi)

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'kcs'
        odom.pose.pose = pose_msg
        odom.twist.twist.linear.x,odom.twist.twist.linear.y = u, v
        odom.twist.twist.angular.z = r

        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = pose_msg
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose_stamped)
        
        self.state_pub.publish(odom)
        self.path_pub.publish(self.path_msg)


def main(args=None):
    vessel_params={'L': 230.0, 'B': 32.2, 'd_em': 10.8, 'Cb': 0.651, 'Fn': 0.26, }
    world_params = {'wind_flag': 0, 'wind_speed': np.random.uniform(0, 5), 'wind_dir': np.random.uniform(-np.pi, np.pi), 
                        'wave_flag': 0, 'wave_height': 0, 'wave_period': 0, 'wave_dir': 0}
    rclpy.init(args=args)
    kcs = KCS(vessel_params=vessel_params, world_params=world_params)
    rclpy.spin(kcs)
    kcs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()