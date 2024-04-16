marker_to_color = {
    0: 'w',
    1: 'r',
    2: 'b',
    3: 'g',
    4: 'o',
    5: 'y'
}

def re_orient_cube(top_id, front_id):
    '''
    A funtion that returns the robot move to reorient the cube so that white faces up and green faces robot.
    
        Parameters:
            top_id (int): Tag ID of top face
            front_id (int): Tag ID of camera side face
        Retirns:
            (list): RObot moves to re-orient the cube
    '''
    top_color = marker_to_color[top_id]
    front_color = marker_to_color[front_id]
    value = top_color + front_color
    mapper = {
        'wg': "y'",
        'wr': "y2",
        'wb': "y",
        'wo': "",

        'gw': 'z',
        'go': "x'",
        'gy': "z'",
        'gr': "x",

        'rg': "x' y'",
        'ry': "z' y'",
        'rb': "x y'",
        'rw': "z y'",

        'bo': "x y2",
        'bw': 'z y2',
        'br': "x' y2",
        'by': "z' y2",

        'yg': "z2 y'",
        'yo': "z2",
        'yb': 'z2 y',
        'yr': 'z2 y2',

        'ob': "x' y",
        'oy': "z' y",
        'og': "x y",
        'ow': "z y",
    }

    return mapper[value].split(' ')


    
