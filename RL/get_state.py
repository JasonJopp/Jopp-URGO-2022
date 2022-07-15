import math

def get_state(self):

    # Calculate state based on column-position of the blob centroid

    # copy left and right end points of object.
    left_point = [self.target_left_edge[0], self.target_left_edge[2]]
    right_point = [self.target_right_edge[0], self.target_right_edge[2]]

    # Determines if front of object is behind the camera
    if left_point[1] < 0 and right_point[1] < 0:
        return self.fail_state

    # if here, we know not both of them are behind the camera.
    # maybe one of them is ...
    if left_point[2] < 0:
        left_point[2] = 0

    elif right_point[2] < 0:
        right_point[2] = 0

    # calculate center point of blob
    center_x = (left_point[0] + right_point[0])/2
    center_z = (left_point[1] + right_point[1])/2

    # calculate left-most and right-most visible point at depth center_z
    # these are the x-coordinates relative to the camera
    left_border = center_z * math.tan(-viewing_angle/2)
    right_border = center_z * math.tan(viewing_angle/2)

    # Determines if object is not in view
    if right_point[0] < left_border:
        return self.fail_state

    if left_point[0] > right_border:
        return self.fail_state

    # we now some portion of the object is in view, but ...
    # determine if center of object is not in viewing angle
    # if not, use the border as the end point of the object
    if not (left_border <= center_x <= right_border):
        if left_point[0] < left_border:
            center_x = (left_border + right_point[0])/2
        else:
            center_x = (left_point[0] + right_border)/2

    mm_per_div = (right_border - left_border)/self.image_divisions
    state = (center_x + left_border) // mm_per_div

    return state
