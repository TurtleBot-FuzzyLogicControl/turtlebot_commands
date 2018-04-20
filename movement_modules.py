from __future__ import print_function
import numpy as np
# from create_env import TurtleBot_ROS

class Movement:

    def __init__(self, goal_position):

        self.lin_velo_x = 0.2
        self.lin_velo_y = 0.2
        self.ang_velo_z = 0.2
        self.goal = goal_position

    def align_heading(self, current_pos):

        est_dist = np.sqrt((self.goal[0] - current_pos[0])**2 + (self.goal[1] - current_pos[1])**2)
        est_angle  = np.arctan2((self.goal[1] - current_pos[1]), (self.goal[0] - current_pos[0]))
        return  est_dist, est_angle

    def membership_functions_TFLC(self, dist, angle_dev):

        # Distance - Zero:z, Very Near: vn, Near:n, Mid-way:m, Far:f, Very Far:vf
        mem_dist = {'z': 0.4, 'vn':0.5, 'n':0.6, 'm':0.8, 'f':1.0, 'vf':1.3}
        mem_dist_keys = ['z', 'vn', 'n', 'm', 'f', 'vf']

        # Angular Position - Neg Opp: no, Neg Right-Angled: nr, Neg Thirty: nt, Neg Five:nf, Aligned: a
        #                    Pos Five:rf, Pos Thirty: rt, Pos Right-Angled: rr, Pos Opp: ro
        mem_angle_dev = {'no': -180, 'nr': -90, 'nt': -30, 'nf': -5, 'a': 0, 'rf': 5, 'rt':30, 'rr':90, 'ro':180}
        mem_angle_dev_keys = ['no', 'nr', 'nt', 'nf', 'a', 'rf', 'rt', 'rr', 'ro']

        nan = 'nan'
        # print (dist, angle_dev)

        def membership_dist():

            prev_key = None
            lenL = len(mem_dist_keys)

            for index, key in enumerate(mem_dist_keys):

                # print ("Index", index)
                # print ("Key", key)
                # print("Mem val", mem_dist[key])

                if index==0 and dist <= mem_dist[key]:
                    return 1, key, nan
                elif index== lenL-1 and dist>mem_dist[mem_dist_keys[lenL-1]]:
                    return 1, mem_dist_keys[lenL-1], nan
                elif dist<=mem_dist[key] and dist>mem_dist[prev_key]:
                    return ((dist - mem_dist[prev_key]) / (mem_dist[key] - mem_dist[prev_key])), key, prev_key
                prev_key = key

        def membership_angle():

            pprev_key = None
            lenLL = len(mem_angle_dev_keys)

            for index, key in enumerate(mem_angle_dev_keys):

                # print ("Index", index)
                # print ("Key", key)
                # print("Mem val", mem_angle[key])

                if index==0 and angle_dev <= mem_angle_dev[key]:
                    return 1, key, nan
                elif index == lenLL-1 and angle_dev>mem_angle_dev[mem_angle_dev_keys[lenLL-1]]:
                    return 1, mem_angle_dev_keys[lenLL-1], nan
                elif angle_dev<=mem_angle_dev[key] and angle_dev>mem_angle_dev[pprev_key]:
                    return ((angle_dev-mem_angle_dev[pprev_key])/float(mem_angle_dev[key] - mem_angle_dev[pprev_key])), key, pprev_key
                pprev_key = key

        mdd = membership_dist()
        maa = membership_angle()

        # print ("Membership function for distance", mdd)
        # print ("Membership function for angle", maa)

        return mdd, maa

    def membership_functions_OAFLC(self, obs_dist, obs_angle):

        # Distance - Zero:z, Near:n, Mid-way:m, Far:f, Very Far:vf
        mem_obs_dist = {'z': 0.4, 'n':0.6, 'm':0.8, 'f':1.0, 'vf':1.3}
        mem_obs_dist_keys = ['z', 'n', 'm', 'f', 'vf']

        # Angular Position - Left max: lm, LeftA: la, LeftB: lb, LeftC:lc, LeftD: ld, Zero: z
        #                    RightD: rd, RightC:rc, RightB: rb, RightA: ra, Right max: rm
        mem_obs_angle_dev = {'lm':-30, 'la':-24, 'lb':-18, 'lc':-12, 'ld': -6, 'z': 0, 'rd':6, 'rc':12, 'rb':18, 'ra':24, 'rm':30}
        mem_obs_angle_dev_keys = ['lm','la','lb','lc','ld','z','rd','rc','rb','ra','rm']

        nan = 'nan'

        # print (obs_dist, obs_angle)

        def membership_obs_dist():

            prev_key = None
            lenL = len(mem_obs_dist_keys)

            for index, key in enumerate(mem_obs_dist_keys):

                # print ("Index", index)
                # print ("Key", key)
                # print("Mem val", mem_dist[key])
                if index==0 and obs_dist <= mem_obs_dist[key]:
                    return 1, key, nan
                elif index== lenL-1 and obs_dist>mem_obs_dist[mem_obs_dist_keys[lenL-1]]:
                    return 1, mem_obs_dist_keys[lenL-1], nan
                elif obs_dist<=mem_obs_dist[key] and obs_dist>mem_obs_dist[prev_key]:
                    return ((obs_dist - mem_obs_dist[prev_key]) / (mem_obs_dist[key] - mem_obs_dist[prev_key])), key, prev_key
                prev_key = key

        def membership_obs_angle():

            pprev_key = None
            lenLL = len(mem_obs_angle_dev_keys)

            for index, key in enumerate(mem_obs_angle_dev_keys):

                # print ("Index", index)
                # print ("Key", key)
                # print("Mem val", mem_angle[key])

                if index==0 and obs_angle <= mem_obs_angle_dev[key]:
                    return 1, key, nan
                elif index == lenLL-1 and obs_angle > mem_obs_angle_dev[mem_obs_angle_dev_keys[lenLL-1]]:
                    return 1, mem_obs_angle_dev_keys[lenLL-1], nan
                elif obs_angle <= mem_obs_angle_dev[key] and obs_angle > mem_obs_angle_dev[pprev_key]:
                    return ((obs_angle - mem_obs_angle_dev[pprev_key])/float(mem_obs_angle_dev[key] - mem_obs_angle_dev[pprev_key])), key, pprev_key
                pprev_key = key

        mdd_a = membership_obs_dist()
        maa_a = membership_obs_angle()

        print ("Membership function for distance for obstacle", mdd_a)
        print ("Membership function for angle for obstacle", maa_a)

        return mdd_a, maa_a

    def membership_functions_FUSION(self, weight_tau):
        '''Defining membership functions for behavior fusion of the osbtacle avoidance and the tracking FLC systems'''

        # This function basically uses the same membership as does the OAFLC as the distribution of the same is
        # responsible for determining how much value is to be given to the fusion system
        nan = 'nan'

        mfw_OAFLC = {'small':0.25, 'medium':0.50, 'large':0.75}
        mfw_OAFLC_keys = ['small', 'medium', 'large']
        def calc():
            if weight_tau <= mfw_OAFLC[mfw_OAFLC_keys[0]]:
                return 1, mfw_OAFLC_keys[0], nan
            elif weight_tau > mfw_OAFLC[mfw_OAFLC_keys[-1]]:
                return 1, mfw_OAFLC_keys[-1], nan
            elif weight_tau <= mfw_OAFLC[mfw_OAFLC_keys[1]] and weight_tau > mfw_OAFLC[mfw_OAFLC_keys[0]]:
                return ((weight_tau - mfw_OAFLC[mfw_OAFLC_keys[0]])/(mfw_OAFLC[mfw_OAFLC_keys[1]] - mfw_OAFLC[mfw_OAFLC_keys[0]])), mfw_OAFLC_keys[1],\
                                                                                                                                mfw_OAFLC_keys[0]
            elif weight_tau <= mfw_OAFLC[mfw_OAFLC_keys[2]] and weight_tau > mfw_OAFLC[mfw_OAFLC_keys[1]]:
                return ((weight_tau - mfw_OAFLC[mfw_OAFLC_keys[1]])/(mfw_OAFLC[mfw_OAFLC_keys[2]] - mfw_OAFLC[mfw_OAFLC_keys[1]])), mfw_OAFLC_keys[2],\
                                                                                                                                mfw_OAFLC_keys[1]
        mf_fusion = calc()
        print ("Membership function for behavior fusion", mf_fusion)

        return mf_fusion

    def behavior_fusion(self, values_tflc, values_oaflc):
        '''Fusing  the individual behaviors of the TFLC and the OAFLC

        values_tflc     - linear X, linear Y and angular Z velocities finally provided by the TFLC
        values_oaflc    - linear X, linear Y and angular Z velocities finally provided by the OAFLC
        '''

        # Need to call the weight deciding function
        weights = None
        w = 0.62

        values_final = [ w*a - (1-w)*b for a,b in zip(values_oaflc, values_tflc)]

        # Final values of the linear X, linear Y and  angular Z velocities
        print ("Final values of veloccities and acccelerations going to the TurtleBot", values_final)

        return values_final


if __name__ == '__main__':

    obje = Movement((6,9))
    a, b = obje.membership_functions_TFLC(0.82, -42)
    c, d = obje.membership_functions_OAFLC(0.79, -7.5)
    e = obje.membership_functions_FUSION(0.65)





