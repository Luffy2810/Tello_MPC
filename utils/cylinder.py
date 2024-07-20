import numpy as np

def convert_2D_3D(point,depth,fx_d,fy_d,cx_d,cy_d):
	x,y = point 
	x_3d = (x_d - cx_d) * depth / fx_d
	y_3d = (y_d - cy_d) * depth / fy_d
	z_3d = depth
	point_3d = np.array(x_3d,y_3d,z_3d)
	return point_3d

def closest_point_on_cylinder(camera_pos, bbox, depth):

    x, y, width, height = bbox
    
    radius = width / 2
    cylinder_height = height
    cylinder_center = convert_2D_3D([x + width/2, y + height/2], depth)
    
    cam_to_center = cylinder_center - camera_pos
    
    projection = cam_to_center - np.dot(cam_to_center, [0,0,1]) * np.array([0, 0, 1])
    
    projection_length = np.linalg.norm(projection)
    if projection_length > 0:
        projection_normalized = projection / projection_length
    else:
        projection_normalized = np.array([1, 0, 0])
    
    closest_on_circle = cylinder_center + projection_normalized * min(radius, projection_length)
    
    z_offset = np.clip(camera_pos[2] - cylinder_center[2], 0, cylinder_height)
    closest_point = closest_on_circle + np.array([0, 0, z_offset])
    
    return closest_point

camera_position = np.array([0, 0, 0])  
bounding_box = (100, 50, 50, 180)  
depth = 50

h = np.linalg.norm(c - p) - (rho + d_s)**2
delta_h = h - h_old  # constraint: h > = gamma*delta_h

closest_point = closest_point_on_cylinder(camera_position, bounding_box, depth)
print(f"Closest point on cylinder: {closest_point}")