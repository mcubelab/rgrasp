import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
    
#########################################################
# orion
# Description:
#~2017 projection
#
#~Usage
#~ suction_projection(bin_pts,hand_pts,target_wf,target_hf,theta,show_plot=False,margin=0)
#
#~Parameters:
#
#~Inputs:
# 1) bin_pts: Points denoting the convex hull of the bin
# 2) hand_pts: Points denoting the convex hull of the hand
# 3) target_wf: Desired position in the world frame
# 4) target_hf: Point on the hand frame the we want to align with target_wf after the hand has rotated by theta then translated by shape_translation
#               however, since we are constrained to make sure that the result is collision free, target_wf may not align with target_hf at the end :(
#               the solution will minimize this error (dist_val_min)
# 5) theta: Rotation of the hand in the x-y plane
# 6) margin: Minimum distance from the bin wall that we want the hand to go
#
#~Output:
# 1) shape_translation: Translation of the hand
# 2) dist_val_min: Distance between target_wf and target_hf at the projected hand position
# 3) feasible: True if a collision free solution exists
# 4) nearest_point: Location of target_hf once the hand has rotated by theta then translated by shape_translation
#########################################################


def projection_func(bin_pts,hand_pts,target_wf,target_hf,theta,show_plot=False,margin=0):
    rot_theta = np.vstack([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    hand_pts = np.transpose(np.dot(rot_theta,np.transpose(hand_pts)))
    target_hf = np.transpose(np.dot(rot_theta,np.transpose(target_hf)))
    bin_hull = ConvexHull(bin_pts)
    bin_mean = np.array([np.mean(bin_pts[:,0]),np.mean(bin_pts[:,1])],ndmin=2)
    hand_mean = np.array([np.mean(hand_pts[:,0]),np.mean(hand_pts[:,1])],ndmin=2)
    mean_diff=bin_mean-hand_mean
    target_net=target_wf-target_hf
    rot90=np.vstack([[0,1.0],[-1.0,0]])
    
    l_bin_hull=len(bin_hull.vertices)
    
    sv_norm_list=[]
    sv_c_list=[]
    index_list=[]
    feas_A=[]
    feas_B=[]
    for bin_count in range(0,l_bin_hull):
        bv0=bin_hull.vertices[bin_count]
        bv1=bin_hull.vertices[(bin_count+1)%l_bin_hull]
        edge=bin_pts[bv0, :]-bin_pts[bv1, :]
        edge_midpoint=(bin_pts[bv0, :]+bin_pts[bv1, :])/2
        bin_simplex_normal = np.dot(rot90,edge)
        bin_simplex_normal=bin_simplex_normal/np.linalg.norm(bin_simplex_normal)

        #plt.plot([edge_midpoint[0],edge_midpoint[0]+.1*bin_simplex_normal[0]],[edge_midpoint[1],edge_midpoint[1]+.1*bin_simplex_normal[1]],'k-')
        
        cmin=None
        min_index=None
        for count in range(0,hand_pts.shape[0]):
            if cmin==None:
                cmin=np.dot(hand_pts[count,:]-edge_midpoint,bin_simplex_normal)-margin
                min_index=count
            else:
                cmin_temp=np.dot(hand_pts[count,:]-edge_midpoint,bin_simplex_normal)-margin
                if cmin_temp<cmin:
                    cmin=cmin_temp
                    min_index=count
                
        #plt.plot([hand_pts[min_index,0],hand_pts[min_index,0]+.1*bin_simplex_normal[0]],[hand_pts[min_index,1],hand_pts[min_index,1]+.1*bin_simplex_normal[1]],'k-')		
        feas_A.append(bin_simplex_normal)
        feas_B.append(cmin)
        if np.dot(target_net,bin_simplex_normal)+cmin<=0:
            sv_norm_list.append(bin_simplex_normal)
            sv_c_list.append(cmin)
            #plt.plot([hand_pts[min_index,0],hand_pts[min_index,0]+.1*bin_simplex_normal[0]],[hand_pts[min_index,1],hand_pts[min_index,1]+.1*bin_simplex_normal[1]],'r-')
            index_list.append(bin_count)
    

    feas_A=np.vstack(feas_A)
    feas_B=np.array(feas_B)
    shape_translation=None
    feasible_solution=False
    if len(sv_c_list)==0:
        shape_translation=target_net[0,:]
        dist_val_min=0
        feasible_solution=True
    if len(sv_c_list)==1:
        (shape_translation,dist_val_min)=project_closest(sv_norm_list[0],sv_c_list[0],target_net)
        feasible_solution= min(np.dot(feas_A,shape_translation)+feas_B)>=0
        #feasible_solution=True
    if len(sv_c_list)>=2:
        dist_val_min=None
        shape_translation=None
        sv_norm_list.append(sv_norm_list[0])
        sv_c_list.append(sv_c_list[0])
        index_list.append(index_list[0])
        sv_norm_list.append(sv_norm_list[1])
        sv_c_list.append(sv_c_list[1])
        index_list.append(index_list[1])
        for count in range(1,len(sv_c_list)-1):
            (shape_translationA,dist_valA,feasible0)=project_intersection(sv_norm_list[count],sv_c_list[count],sv_norm_list[count+1],sv_c_list[count+1],target_net)
            (shape_translationB,dist_valB)=project_closest(sv_norm_list[count],sv_c_list[count],target_net)
            
            if feasible0:
                feasible1= min(np.dot(feas_A,shape_translationA)+feas_B)>=0
                if dist_val_min==None and feasible1:
                    dist_val_min=dist_valA
                    shape_translation=shape_translationA
                    feasible_solution=True
                if dist_valA<dist_val_min and feasible1:
                    dist_val_min=dist_valA
                    shape_translation=shape_translationA

            feasible2= min(np.dot(feas_A,shape_translationB)+feas_B)>=0
            if dist_val_min==None and feasible2:
                dist_val_min=dist_valB
                shape_translation=shape_translationB
                feasible_solution=True
            if dist_valB<dist_val_min and feasible2:
                dist_val_min=dist_valB
                shape_translation=shape_translationB
                
    nearest_point=None
    if feasible_solution:
        nearest_point=np.array([target_hf[:,0]+shape_translation[0], target_hf[:,1]+shape_translation[1]])
    if show_plot:
        hand_hull = ConvexHull(hand_pts)
        for simplex in bin_hull.simplices:
            plt.plot(bin_pts[simplex, 0], bin_pts[simplex, 1], 'k-')
        for simplex in hand_hull.simplices:
            plt.plot(hand_pts[simplex, 0]+mean_diff[:,0], hand_pts[simplex, 1]+mean_diff[:,1], 'k-')
        plt.plot(bin_pts[:,0], bin_pts[:,1], 'bo')
        plt.plot(hand_pts[:,0]+mean_diff[:,0], hand_pts[:,1]+mean_diff[:,1], 'ro')
        
        if feasible_solution:
            plt.plot(hand_pts[:,0]+shape_translation[0], hand_pts[:,1]+shape_translation[1], 'ro')
            for simplex in hand_hull.simplices:
                plt.plot(hand_pts[simplex, 0]+shape_translation[0], hand_pts[simplex, 1]+shape_translation[1], 'k-')
            plt.plot(target_hf[:,0]+shape_translation[0], target_hf[:,1]+shape_translation[1], 'go')
        plt.plot(target_wf[:,0], target_wf[:,1], 'co')
        plt.plot(target_hf[:,0]+mean_diff[:,0], target_hf[:,1]+mean_diff[:,1], 'go')
        
        plt.axis('equal')
        plt.show()
    return (shape_translation,dist_val_min,feasible_solution,nearest_point)


def project_closest(n1,c1,p): 
    n1=n1/np.linalg.norm(n1)
    p_new=p-n1*(np.dot(p,n1)+c1)
    dist_out=np.linalg.norm(p-p_new)
    p_new=p_new[0,:]
    return p_new,dist_out

def project_intersection(n1,c1,n2,c2,p):
    A=np.vstack([n1,n2])
    B=-np.array([c1,c2])
    try:
        p_new=np.linalg.solve(A,B)
        dist_out=np.linalg.norm(p-p_new)
        return p_new,dist_out,True
    except:
        return None,None,False

def optimize_angle(bin_pts,hand_pts,target_wf,target_hf,theta_list,show_plot=True,margin=0):
    dist_val_min=None
    shape_translation=None
    found_feasible=False
    theta_min=None
    nearest_point=None
    for theta in theta_list:
        (shape_translation_temp,dist_val,feasible_solution,temp_nearest)=suction_projection_func(bin_pts,hand_pts,target_wf,target_hf,theta,show_plot=False,margin=margin)
        if feasible_solution and (dist_val_min==None or dist_val<dist_val_min):
            found_feasible=True
            dist_val_min=dist_val
            shape_translation=shape_translation_temp
            theta_min=theta
            nearest_point=temp_nearest
    if show_plot and found_feasible:
        suction_projection_func(bin_pts,hand_pts,target_wf,target_hf,theta_min,show_plot=True,margin=margin)
    #print theta_min
    return (shape_translation,dist_val,found_feasible,nearest_point,theta_min)
    
    

if __name__=="__main__":
    #bin_pts=[[1.5,0],[2,1],[.9,1.6],[0,1],[0,0],[.7,-.3],[.7,-.3]]
    #bin_pts=np.vstack(bin_pts)
    #bin_pts_range=range(0,bin_pts.shape[0])
    #bin_pts_range.append(0) 
    
    #hand_pts=[[-.5,.25],[0,0],[1,2],[.5,2.25]]
    #hand_pts=[[-.5,.25]]
    #hand_pts=np.vstack(hand_pts)
    
    #hand_pts=.4*hand_pts+np.matlib.repmat([.5,.2], hand_pts.shape[0],1)

    #hand_pts_range=range(0,hand_pts.shape[0])
    #hand_pts_range.append(0) 

    #target_wf=np.array([-.1,-.1],ndmin=2)
    #target_wf=np.array([.01,1.2],ndmin=2)
    #target_wf=np.array([-.2,1.1],ndmin=2)
    #target_wf=np.array([.15,1.3],ndmin=2)
    #target_wf=np.array([-.25,.5],ndmin=2)
    #target_wf=np.array([1.8,.3],ndmin=2)
    #target_wf=np.array([1.6,-.1],ndmin=2)
    #target_wf=np.array([1.8,1.2],ndmin=2)
    #target_wf=np.array([1.5,1.5],ndmin=2)
    #target_wf=np.array([.25,-.2],ndmin=2)
    #target_wf=np.array([.56,.58],ndmin=2)
    #target_wf=np.array([.3,.4],ndmin=2)
    #target_hf=np.array([.75,1],ndmin=2)
    #target_hf=hand_pts
    #theta=.5
	#(shape_translation,dist_val,feasible_solution,theta_min)=optimize_angle(bin_pts,hand_pts,target_wf,target_hf,np.linspace(0,2*np.pi,50),show_plot=True,margin=.3)

	#EXAMPLE USAGE!!!
    #bin_pts=[[1.5,0],[2,1],[.9,1.6],[0,1],[0,0],[.7,-.3],[.7,-.3]]
    #bin_pts=np.vstack(bin_pts)
    #hand_pts=[[-.5,.25],[0,0],[1,2],[.5,2.25]]
    #hand_pts=np.vstack(hand_pts)
    #hand_pts=.4*hand_pts+np.matlib.repmat([.5,.2], hand_pts.shape[0],1)
    #target_wf=np.array([1.8,1.2],ndmin=2)
    #target_hf=np.array([.75,1],ndmin=2)
    #theta=.5
    #show_plot=True
    #margin=0

    #Debug usage:

    bin_pts = [[ 1.203355, -0.04008 ],
        [ 1.203355, -0.04008 ],
        [ 0.758355, -0.04008 ],
        [ 0.758355, -0.04008 ],
        [ 0.758355, -0.35008 ],
        [ 0.758355, -0.35008 ],
        [ 1.203355, -0.35008 ],
        [ 1.203355, -0.35008 ]]
    bin_pts=np.vstack(bin_pts)
    #hand_pts = [[-0.0206715, 0.0429375],
    #    [-0.0206715, 0.0429375],
    #    [-0.0206715, -0.0429375],
    #    [-0.0206715, -0.0429375],
    #    [ 0.015, 0.0429375],
    #    [ 0.015, 0.0429375],
    #    [ 0.015, -0.0429375],
    #    [ 0.015, -0.0429375],
    #    [-0.838855, 0.17158 ],
    #    [-0.838855, 0.17158 ],
    #    [-0.838855, 0.21858 ],
    #    [-0.838855, 0.21858 ],
    #    [-1.157855, 0.17158 ],
    #    [-1.157855, 0.17158 ],
    #    [-1.157855, 0.21858 ],
    #    [-1.157855, 0.21858 ]]
    hand_pts=np.array([-0.998355, 0.19508 ],ndmin=2)
    #hand_pts=np.vstack(hand_pts)
    target_wf = np.array([ 0.858355, -0.21008 ],ndmin=2)
    target_hf = np.array([-0.998355, 0.19508 ],ndmin=2)
    theta = -7.89909804234e-12
    show_plot = False
    margin = 0.035

    (shape_translation,dist_val_min,feasible_solution,nearest_point)=suction_projection_func(bin_pts,hand_pts,target_wf,target_hf,theta,show_plot,margin)

    
    if feasible_solution:
        print 'feasible solution found :)'
    else:
        print 'no feasible solution found :('


