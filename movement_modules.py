import numpy as np


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

        # Angular Position - Neg Opp: no, Neg Right-Angled: nr, Neg Thirty: nt, Neg Five:nf, Aligned: a
        #                    Pos Five:rf, Pos Thirty: rt, Pos Right-Angled: rr, Pos Opp: ro
        mem_angle_dev = {'no': -180, 'nr': -90, 'nt': -30, 'nf': -5, 'a': 0, 'rf': 5, 'rt':30, 'rr':90, 'ro':180}

        print (dist, angle_dev)

        def membership_dist():

            if dist <= mem_dist['z']:
                md = 1
            elif dist <=  mem_dist['vn'] and dist > mem_dist['z']:
                md = (dist - mem_dist['z'])/ (mem_dist['vn'] - mem_dist['z'])
            elif dist <=  mem_dist['n'] and dist > mem_dist['vn']:
                md = (dist - mem_dist['vn'])/ (mem_dist['n'] - mem_dist['vn'])
            elif dist <=  mem_dist['m'] and dist > mem_dist['n']:
                md = (dist - mem_dist['n'])/ (mem_dist['m'] - mem_dist['n'])
            elif dist <=  mem_dist['f'] and dist > mem_dist['m']:
                md = (dist - mem_dist['m'])/ (mem_dist['f'] - mem_dist['m'])
            elif dist <=  mem_dist['vf'] and dist > mem_dist['f']:
                md = (dist - mem_dist['f'])/ (mem_dist['vf'] - mem_dist['f'])
            elif dist > mem_dist['vf']:
                md = 1

            return md

            # prev_key = None
            # lenL = len(list(mem_dist.keys()))
            #
            # for index, key in enumerate(list(mem_dist.keys())):
            #     if index==1 and dist <= mem_dist[key]:
            #         return 0, key
            #     elif index== lenL-1 and dist>mem_dist[list(mem_dist.keys())[lenL-1]]:
            #         return 0, list(mem_dist.keys())[lenL-1]
            #     elif dist<=mem_dist[key] and dist>mem_dist[prev_key]:
            #         return ((dist - mem_dist[prev_key]) / (mem_dist[key] - mem_dist[prev_key])), prev_key, key
            #     prev_key = key


        def membership_angle():

            if angle_dev <= mem_angle_dev['no']:
                ma = 1

            elif angle_dev <= mem_angle_dev['nr'] and angle_dev > mem_angle_dev['no']:
                ma = (angle_dev - mem_angle_dev['no']) / (mem_angle_dev['nr'] - mem_angle_dev['no'])

            elif angle_dev <= mem_angle_dev['nt'] and angle_dev > mem_angle_dev['nr']:
                ma = (angle_dev - mem_angle_dev['nr']) / (mem_angle_dev['nt'] - mem_angle_dev['nr'])

            elif angle_dev <= mem_angle_dev['nf'] and angle_dev > mem_angle_dev['nt']:
                ma = (angle_dev - mem_angle_dev['nt']) / (mem_angle_dev['nf'] - mem_angle_dev['nt'])

            elif angle_dev <= mem_angle_dev['a'] and angle_dev > mem_angle_dev['nf']:
                ma = (angle_dev - mem_angle_dev['nf']) / (mem_angle_dev['a'] - mem_angle_dev['nf'])

            elif angle_dev <= mem_angle_dev['rf'] and angle_dev > mem_angle_dev['a']:
                ma = (angle_dev - mem_angle_dev['a']) / (mem_angle_dev['rf'] - mem_dist['a'])

            elif angle_dev <= mem_angle_dev['rt'] and angle_dev > mem_angle_dev['rf']:
                ma = (angle_dev - mem_angle_dev['rf']) / (mem_angle_dev['rt'] - mem_angle_dev['rf'])

            elif angle_dev <= mem_angle_dev['rr'] and angle_dev > mem_angle_dev['rt']:
                ma = (angle_dev - mem_angle_dev['rt']) / (mem_angle_dev['rr'] - mem_angle_dev['rt'])

            elif angle_dev <= mem_angle_dev['ro'] and angle_dev > mem_angle_dev['rr']:
                ma = (angle_dev - mem_angle_dev['rr']) / (mem_angle_dev['ro'] - mem_angle_dev['rr'])

            elif angle_dev > mem_angle_dev['ro']:
                ma = 1

            return ma


        mdd = membership_dist()
        maa = membership_angle()

        print ("Membership function for distance", mdd)
        print ("Membership function for angle", maa)

if __name__ == '__main__':

    obje = Movement((6,9))
    obje.membership_functions_TFLC(0.45, 15)

