# -*- coding: utf-8 -*-
"""
Created on Mon Jun 11 15:46:28 2018

@author: dennis
"""
#======================================================
#Original Code taken from Bluesky GitHup
#Authors: J.M.Hoesktra, J.Ellerbroek and S.Balassoryian
 
#ADDITIONAL CODE ADDED BY
#Author: Leonor Inverno
 
#CONVERTED CODE FOR DRONE PURPOSES (FOR PAPARAZZIUAV) DONE BY
#Author: Dennis van Wijngaarden
#======================================================
 
import numpy as np
import traffic
import geo
import sys

try:
    import pyclipper
except ImportError:
    print("Could not import pyclipper, RESO SSD will not function")

#Conversion variables
nm = 1852 #NM to m
ft = 0.3048 #ft to m
kts = 0.514444 #kts to m/s

#------------------------------------------------------------------
#General functions
#------------------------------------------------------------------
        
#-----------------------------------------------------------------------------
# Functions adapted from bluesky/asas/SSD.py
#-----------------------------------------------------------------------------

#This class is created to be easier to adapt code and also to make 
#variables global
        
class Asas:
    def __init__(self, ntraf, V, strategy, tla, margin):
        
        #Look-ahead time to be considered for conflict resolution
        self.tla = tla #five minutes look-ahead time
        
        #Forbidden Reachable Velocity regions
        self.FRV          = [None] * ntraf
        
        #Allowed Reachable Velocity regions
        self.ARV          = [None] * ntraf
        self.ARV_calc     = [None] * ntraf

        self.inrange      = [None] * ntraf
        self.inconf       = np.zeros(ntraf, dtype=bool)

        # Stores resolution vector, also used in visualization
        self.asasn        = np.zeros(ntraf, dtype=np.float32)
        self.asase        = np.zeros(ntraf, dtype=np.float32)
        # Area calculation
        self.FRV_area     = np.zeros(ntraf, dtype=np.float32) 
        
        self.ARV_area     = np.zeros(ntraf, dtype=np.float32)
        
        self.free_tla       = np.ones(ntraf, dtype=bool) #conflict free at a given look-ahead time
        
        #To store the new track and velocity after resolution
        self.trk        = [None] * ntraf
        self.tas        = [None] * ntraf
        
        
        self.vmin = 5. # [m/s]
        self.vmax = 40. # [m/s]
        self.v_air_nom = 14. # [m/s]
        #self.R = 5*nm #minimum horizontal separation of 5NM (protected zone)
        self.mar = margin
        
        self.strategy = strategy
    
def qdrdist_matrix_indices(ntraf):
    """ This function gives the indices that can be used in the lon/lat-vectors """
    # The indices will be n-1 long
    # Only works for n >= 2, which is logical...
    # This is faster than np.triu_indices :)
    ind1 = np.zeros(ntraf-1).astype(np.int32)
    ind2 = np.arange(1,ntraf).astype(np.int32)
    return ind1, ind2  

def area(vset):
    """ This function calculates the area of the set of FRV or ARV """
    # Initialize A as it could be calculated iteratively
    A = 0
    # Check multiple exteriors
    if type(vset[0][0]) == list:
        # Calc every exterior separately
        for i in range(len(vset)):
            A += pyclipper.scale_from_clipper(pyclipper.scale_from_clipper(pyclipper.Area(pyclipper.scale_to_clipper(vset[i]))))
    else:
        # Single exterior
        A = pyclipper.scale_from_clipper(pyclipper.scale_from_clipper(pyclipper.Area(pyclipper.scale_to_clipper(vset))))
    return A
    
def resolve(asas, traf):
    
    constructSSD(asas, traf, asas.tla)

    # Get resolved speed-vector
    calculate_resolution(asas, traf)

    
    # Now assign resolutions to variables in the ASAS class
    # Start with current states, need a copy, otherwise it changes traf!
    asas.trk = np.copy(traf.hdg)
    asas.tas = np.copy(traf.gs)
    # Calculate new track and speed
    # No need to cap the speeds, since SSD implicitly caps
    new_trk  = np.arctan2(asas.asase, asas.asasn) * 180 / np.pi
    new_tas  = np.sqrt(asas.asase ** 2 + asas.asasn ** 2)

    # Sometimes an aircraft is in conflict, but no solutions could be found
    # In that case it is assigned 0 by ASAS, but needs to handled
    asas_cmd = np.logical_and(asas.inconf, new_tas > 0)

    # Assign new track and speed for those that are in conflict
    asas.trk[asas_cmd] = new_trk[asas_cmd]
    asas.tas[asas_cmd] = new_tas[asas_cmd]
    
    return
    
def constructSSD(asas, traf, tla, wind):
    """ Calculates the FRV and ARV of the SSD """
    
    # Parameters
    N_angle = 180                  # [-] Number of points on circle (discretization)
    vmin    = asas.vmin             # [m/s] Defined in asas.py
    vmax    = asas.vmax             # [m/s] Defined in asas.py
    #hsep    = asas.R                # [m] Horizontal separation (5 NM)
    margin  = asas.mar              # [-] Safety margin for evasion
    #hsepm   = hsep * margin         # [m] Array of horizontal separation with safety margin
    alpham  = 0.4999 * np.pi        # [rad] Maximum half-angle for VO
    betalos = np.pi / 4             # [rad] Minimum divertion angle for LOS (45 deg seems optimal)
    adsbmax = 250. * nm             # [m] Maximum ADS-B range PRIOR: 65NM
    beta    =  np.pi/4 + betalos/2
    
    # Relevant info from traf
    gsnorth = traf.gsnorth          #North component of all velocity vectors
    gseast  = traf.gseast           #East component of all velocity vectors
    lat     = traf.lat              #All latitudes
    lon     = traf.lon              #All Longitudes
    ntraf   = traf.ntraf            #Number or aircraft involved
    
    # Traffic seperation exlcuding seperation with itself
    hsep    = traf.hsep[1:]            # [m] Array of horizontal seperation minima (depending on traffic type)
    hsepm   = hsep + margin         # [m] Array of horizontal separation with safety margin
    
    # Local variables, will be put into asas later
    FRV_loc          = [None] * traf.ntraf
    ARV_loc          = [None] * traf.ntraf
    ARV_calc_loc     = [None] * traf.ntraf
    FRV_area_loc     = np.zeros(traf.ntraf, dtype=np.float32)
    ARV_area_loc     = np.zeros(traf.ntraf, dtype=np.float32)
    
    # # Use velocity limits for the ring-shaped part of the SSD
    # Discretize the circles using points on circle
    angles = np.arange(0, 2 * np.pi, 2 * np.pi / N_angle)
    # Put points of unit-circle in a (180x2)-array (CW)
    xyc = np.transpose(np.reshape(np.concatenate((np.sin(angles), np.cos(angles))), (2, N_angle)))
    # Map them into the format pyclipper wants. Outercircle CCW, innercircle CW
    circle_tup = (tuple(map(tuple, np.flipud(xyc * vmax))), tuple(map(tuple , xyc * vmin)))
    circle_lst = [list(map(list, np.flipud(xyc * vmax))), list(map(list , xyc * vmin))]
    
    # If no traffic
    if ntraf == 0:
        return
    
    # If only one aircraft
    elif ntraf == 1:
        # Map them into the format ARV wants. Outercircle CCW, innercircle CW
        ARV_loc[0] = circle_lst
        ARV_calc_loc[0] = circle_lst
        FRV_loc[0] = []
        
        FRV_area_loc[0] = 0
        ARV_area_loc[0] = np.pi * (vmax **2 - vmin ** 2)
        
        # Added following rules myself to overcome error message
        asas.FRV          = FRV_loc
        asas.ARV          = ARV_loc
        asas.FRV_area     = FRV_area_loc
        asas.ARV_area     = ARV_area_loc
        return
        
    # Function qdrdist_matrix needs 4 vectors as input (lat1,lon1,lat2,lon2)
    # To be efficient, calculate all qdr and dist in one function call
    # Example with ntraf = 5:   ind1 = [0,0,0,0,1,1,1,2,2,3]
    #                           ind2 = [1,2,3,4,2,3,4,3,4,4]
    # This way the qdrdist is only calculated once between every aircraft
    # To get all combinations, use this function to get the indices
    ind1, ind2 = qdrdist_matrix_indices(ntraf)
    
    # Get absolute bearing [deg] and distance [nm]
    # Not sure abs/rel, but qdr is defined from [-180,180] deg, w.r.t. North

    [qdr, dist] = geo.qdrdist_matrix(lat[ind1], lon[ind1], lat[ind2], lon[ind2])

    # Put result of function from matrix to ndarray
    qdr  = np.reshape(np.array(qdr), np.shape(ind1))
    dist = np.reshape(np.array(dist), np.shape(ind1))
    # SI-units from [deg] to [rad]
    qdr  = np.deg2rad(qdr)
    # Get distance from [nm] to [m]
    dist = dist * nm
    
    if any(dist[dist<hsepm]):
        return 'LoS'
    
    # In LoS the VO can't be defined, act as if dist is on edge
    dist[dist < hsepm] = hsepm[dist < hsepm]

    # Calculate vertices of Velocity Obstacle (CCW)
    # These are still in relative velocity space, see derivation in appendix
    # Half-angle of the Velocity obstacle [rad]
    # Include safety margin
    alpha = np.arcsin(hsepm / dist)
    # Limit half-angle alpha to 89.982 deg. Ensures that VO can be constructed
    alpha[alpha > alpham] = alpham 
    #Take the cosinus of alpha to calculate the maximum length of the VO's legs
    cosalpha = np.cos(alpha)
 
    # Calculate SSD only for aircraft 0 when in conflict (See formulas appendix)
    if asas.inconf[0]:
        
        # SSD for aircraft 0, which is the drone
        # Get indices that belong to aircraft 0
        ind = np.where(np.logical_or(ind1 == 0,ind2 == 0))[0]
        # Check whether there are any aircraft in the vicinity
        if len(ind) == 0:
            # No aircraft in the vicinity
            # Map them into the format ARV wants. Outercircle CCW, innercircle CW
            ARV_loc[0] = circle_lst
            ARV_calc_loc[0] = circle_lst
            
            FRV_loc[0] = []
            
            # Calculate areas and store in asas
            FRV_area_loc[0] = 0
            ARV_area_loc[0] = np.pi * (vmax **2 - vmin ** 2)
        else:
            # The i's of the other aircraft
            i_other = np.delete(np.arange(0, ntraf), 0)
            # Aircraft that are within ADS-B range
            ac_adsb = np.where(dist[ind] < adsbmax)[0]
            # Now account for ADS-B range in indices of other aircraft (i_other)
            ind = ind[ac_adsb]
            i_other = i_other[ac_adsb]
            asas.inrange[0]  = i_other
            # VO from 2 to 1 is mirror of 1 to 2. Only 1 to 2 can be constructed in
            # this manner, so need a correction vector that will mirror the VO
            fix = np.ones(np.shape(i_other))
            fix[i_other < 0] = -1

            
            drel_x, drel_y = fix*dist[ind]*np.sin(qdr[ind]), fix*dist[ind]*np.cos(qdr[ind])
            drel = np.dstack((drel_x,drel_y))
            
            cosalpha_i = cosalpha[ind]
                            
            # Make a clipper object
            pc = pyclipper.Pyclipper()
            pc_calc = pyclipper.Pyclipper() #to include resolution with different strategies
            
            # Add circles (ring-shape) to clipper as subject
            pc.AddPath(pyclipper.scale_to_clipper(circle_tup[0]), pyclipper.PT_SUBJECT, True)
            
            #This function returns the IDs of aircraft within tla and the current minimum time to LoS
            for i in range(len(i_other)):
                if i ==0:
                    min_tlos, ids_tla = get_times(asas,traf,0, [i_other[i]], tla)
                else:
                    tlos_new, ids_tla_new = get_times(asas,traf,0, [i_other[i]], tla)
                    ids_tla = np.append(ids_tla,ids_tla_new)
            
            if len(ids_tla) != 0:
                asas.free_tla[0] = False   
        
            for j in range(np.shape(i_other)[0]):
                    
                ## Debug prints
                ##print(traf.id[i] + " - " + traf.id[i_other[j]])
                ## print(dist[ind[j]])
                # Scale VO when not in LOS
                if dist[ind[j]] > hsepm[ind[j]]:
    
                    dist_mod = dist[ind[j]] #the value (not array) of the distance is needed for future computations
                    
                    
                    #direction of the VO's bisector
                    nd = drel[0,j,:]/dist_mod
                    
                    
                    #R_pz = traf.hsep[ind[j]]*asas.mar
                    R_pz = hsep[ind[j]] + asas.mar
                    
                    R = np.array([[np.sqrt(1-(R_pz/dist_mod)**2), R_pz/dist_mod], [-R_pz/dist_mod, np.sqrt(1-(R_pz/dist_mod)**2)] ])
    
                    n_t1 = np.matmul(nd, R) #Direction of leg2
                    
                    n_t2 = np.matmul(nd, np.transpose(R)) #Direction of leg1
                    #VO points
                    v_other = [gseast[i_other[j]],gsnorth[i_other[j]]]
                    legs_length = 8*vmax/cosalpha_i[j] # originally 2*vmac/cosalpha_i[j], changed due to scale problems for drones
                    VO_points = np.array([v_other, np.add(n_t2*legs_length, v_other), np.add( n_t1* legs_length, v_other)])
                    
                    
                    # Normally VO shall be added of this other a/c
                    VO = pyclipper.scale_to_clipper(tuple(map(tuple, VO_points)))
                    
                else:
                    # Pair is in LOS
                    #Store conflict pairs for MVP
                    
                    #Indicate conflict for SSD resolution
                    asas.free_tla[0] = False
                    
                    #Instead of triangular VO, use darttip
                    # Check if bearing should be mirrored
                    if i_other[j] < 0:
                        qdr_los = qdr[ind[j]] + np.pi
                    else:
                        qdr_los = qdr[ind[j]]
                    # Length of inner-leg of darttip
                    leg = 1.1 * vmax / np.cos(beta) * np.array([1,1,1,0])
                    
                    # Angles of darttip
                    angles_los = np.array([qdr_los + 2 * beta, qdr_los, qdr_los - 2 * beta, 0.])
                    # Calculate coordinates (CCW)
                    x_los = leg * np.sin(angles_los)
                    y_los = leg * np.cos(angles_los)
                    # Put in array of correct format
                    xy_los = np.vstack((x_los,y_los)).T
                    # Scale darttip
                    VO = pyclipper.scale_to_clipper(tuple(map(tuple,xy_los)))
                    
                # Add scaled VO to clipper
                pc.AddPath(VO, pyclipper.PT_CLIP, True)
                
            #(end of the for cycle)
            
            # Execute clipper command
            FRV = pyclipper.scale_from_clipper(pc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO))
            ARV = pc.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)

            try:
                pc_calc.AddPaths(ARV, pyclipper.PT_CLIP, True)
            except:
                print("ssd_resolution: Margin to high")
                #return ("margin")
            # Scale back
            ARV = pyclipper.scale_from_clipper(ARV)
            
            if not asas.strategy == "SWO" and not asas.strategy == "DEST":
                #Further restrict the ARV area for other strategies
                if asas.strategy == "HDG":
                    # Small ring
                    
#                    vel_circle = (tuple(map(tuple, np.flipud(xyc * min(vmax, traf.gs[0]+0.001)))), tuple(map(tuple , xyc * max(vmin, traf.gs[0]-0.001))))
                    vel_circle = (tuple(map(tuple, np.flipud(xyc * min(vmax, asas.v_air_nom+0.1) + [wind['east'], wind['north']]))), tuple(map(tuple , xyc * max(vmin, asas.v_air_nom-0.1) + [wind['east'], wind['north']])))
                    pc_calc.AddPaths(pyclipper.scale_to_clipper(vel_circle),pyclipper.PT_SUBJECT, True)
                    ARV_calc = pyclipper.scale_from_clipper(pc_calc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO))
                elif asas.strategy == "SPD":
                    hdg = np.radians(traf.hdg[0])
                    xyp = np.array([[np.sin(hdg-0.0087),np.cos(hdg-0.0087)],
                                        [0,0],
                                        [np.sin(hdg+0.0087),np.cos(hdg+0.0087)]],
                                        dtype=np.float64)
                    vel_line = pyclipper.scale_to_clipper(tuple(map(tuple, 1.1 * vmax * xyp)))
                    pc_calc.AddPath(vel_line,pyclipper.PT_SUBJECT, True)
                    ARV_calc = pyclipper.scale_from_clipper(pc_calc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO))
                    ARV_calc= vel_line
            else:
                ARV_calc = ARV
           
            # Check multi exteriors, if this layer is not a list, it means it has no exteriors
            # In that case, make it a list, such that its format is consistent with further code
           
            if len(ARV) == 0:
                    # No aircraft in the vicinity
                    # Map them into the format ARV wants. Outercircle CCW, innercircle CW
                    ARV_loc[0] = []
                    ARV_calc_loc[0] = []
                    FRV_loc[0] = circle_lst
                    # Calculate areas and store in asas
                    FRV_area_loc[0] = np.pi * (vmax **2 - vmin ** 2)
                    ARV_area_loc[0] = 0
                    
            elif len(FRV) == 0:
                # Should not happen with one a/c or no other a/c in the vicinity.---> remove this when implementing in bluesky
                # These are handled earlier. Happens when RotA has removed all
                # Map them into the format ARV wants. Outercircle CCW, innercircle CW
                
                FRV_loc[0] = []
                
                ARV_loc[0] = circle_lst
                ARV_calc_loc[0] = circle_lst
                
                # Calculate areas and store in asas
                FRV_area_loc[0] = 0
                ARV_area_loc[0] = np.pi * (vmax **2 - vmin ** 2)
                
            else:
                
                if not type(FRV[0][0]) == list:
                    FRV = [FRV]
                    
                if not type(ARV[0][0]) == list:
                    ARV = [ARV]
                    
                if not len(ARV_calc)== 0:
                        if not type(ARV_calc[0][0]) == list:
                            ARV_calc = [ARV_calc]
                
                # Store in asas
                FRV_loc[0] = FRV
                ARV_loc[0] = ARV
                ARV_calc_loc[0] = ARV_calc
                # Calculate areas and store in asas
                FRV_area_loc[0] = area(FRV)
                ARV_area_loc[0] = area(ARV)
                    
                    
                

    #Storing the results into asas
    asas.FRV          = FRV_loc
    asas.ARV          = ARV_loc
    asas.ARV_calc     = ARV_calc_loc
    asas.FRV_area     = FRV_area_loc
    asas.ARV_area     = ARV_area_loc
    return
    
#This function returns the minimum time to LOS of all the conflicts an aircraft is in. Also, it returns
#a list specifying the aircraft within the tla and min_tlos
def get_times(asas, traf, i, i_other, tla):
    
    
    x_other = traf.gseast[i_other]
    y_other = traf.gsnorth[i_other]
    x = traf.gseast[i]
    y = traf.gsnorth[i]
    # Get relative bearing [deg] and distance [nm]
    qdr, dist = geo.qdrdist(traf.lat[i], traf.lon[i], traf.lat[i_other], traf.lon[i_other])
    # Convert to SI
    
    qdr = np.around(qdr, decimals = 0)
    qdr = np.deg2rad(qdr)
    dist *= nm
    
    # For vectorization, store lengths as W and L
    
    #W = np.shape(x)
    L = np.shape(x_other)[0]
    # Relative speed-components
    du = np.dot(x_other.reshape((L,1)),np.ones((1,1))) - np.dot(np.ones((L,1)),x.reshape((1,1)))
    dv = np.dot(y_other.reshape((L,1)),np.ones((1,1))) - np.dot(np.ones((L,1)),y.reshape((1,1)))
    # Relative speed + zero check
    vrel2 = du * du + dv * dv
    vrel2 = np.where(np.abs(vrel2) < 1e-6, 1e-6, vrel2)  # limit lower absolute value
 
    # X and Y distance
    dx = np.dot(np.reshape(dist*np.sin(qdr),(L,1)),np.ones((1,1)))
    dy = np.dot(np.reshape(dist*np.cos(qdr),(L,1)),np.ones((1,1)))
    
    # Time to CPA
    tcpa = -(du * dx + dv * dy) / vrel2
    # CPA distance
    dcpa2 = np.square(np.dot(dist.reshape((L,1)),np.ones((1,1)))) - np.square(tcpa) * vrel2
    
    # Calculate time to LOS
    # R2 = traf.hsep[i_other] * traf.hsep[i_other]
    R2 = (traf.hsep[i_other]+asas.mar) * (traf.hsep[i_other]+asas.mar)
    swhorconf = dcpa2 < R2 #Checks if two aircraft are in conflict
    
    #TLOS expressions
    dxinhor = np.sqrt(np.maximum(0,R2-dcpa2)) #half distance travelled inside the PZ
    dtinhor = dxinhor / np.sqrt(vrel2) #Time in LoS
    tinhor = np.where(swhorconf, tcpa-dtinhor, 0.) #Time to LoS 
    tinhor = np.where(tinhor > 0, tinhor, 1e6) #If two aircraft are not in conflict, then the tLos is set to be a very high number
    tinhor = np.around(tinhor, decimals= 0) #Round final results
    
    if len(tinhor) != 0:
        min_tlos = min(tinhor)[0]
    else:
        min_tlos = 1e6
    
    #Get aircraft within tla
    if tla == 'adsb': #This means that all aircraft are considered regardless of their distance to intruders
        tla = 9e5 #take a very large number but not as high as to consider non-intruding aircraft (1e6)
    
    ids_tla_ind = np.array(np.where(tinhor <= tla + 5))[0,:] #threshold of 1 minute
    indexs_tla = np.take(i_other, ids_tla_ind)
    ids_tla = np.take(traf.id, indexs_tla)
    
    if min_tlos == 1e6:
        #Then the aircraft is not in conflict and therefore no ids or times should be returned 
        return -1, []
    else:
        #Then it returns min tlos ids of aicraft within tla
        return min_tlos, ids_tla
        
#==========================
#Resolution Functions
#==========================
        
def calculate_resolution(asas, traf, tol=0):
    
    # Variables
    ARV = asas.ARV_calc
    
    # Select AP-setting as reference point for closest to target rulesets
    # ntraf   = traf.ntraf
    
    gseast = traf.gseast
    gsnorth = traf.gsnorth
    
    # Only those that are in conflict need to resolve  
    try:
        len(ARV[0])
    except:
        asas.asase[0] = 0.
        asas.asasn[0] = 0.
        print("calculate_resolution: no ARV defined due to pyclip error")
        return('nosol', asas.asase[0],  asas.asasn[0])
        
    if asas.inconf[0] and len(ARV[0]) > 0:
        if asas.free_tla[0] == True:
            asas.asase[0] = gseast[0]
            asas.asasn[0] = gsnorth[0]
            #print("Aircraft", 0, "is free of conflict" )
            return('free', asas.asase[0],  asas.asasn[0])
        else:
            sol_east, sol_north = shortest_way_out(ARV[0], gseast[0], gsnorth[0],0)
            asas.asase[0] = sol_east[0]
            asas.asasn[0] = sol_north[0]
            #print("Aircraft", 0, "has resolved")
            return('conflict', sol_east, sol_north)
    else:    
        asas.asase[0] = 0.
        asas.asasn[0] = 0.
        print("Aircraft", 0, "has not resolved")
        return('nosol', asas.asase[0],  asas.asasn[0])
    
    
    #print(len(shortest_way_out_solutions(asas.ARV[0], gseast[0], gsnorth[0], 0)[0])
        
def shortest_way_out(ARV, gseast, gsnorth,i):
    
    # It's just linalg, however credits to: http://stackoverflow.com/a/1501725
    
    # Loop through all exteriors and append. Afterwards concatenate
    p = []
    q = []
    for j in range(len(ARV)):
        p.append(np.array(ARV[j]))
        q.append(np.diff(np.row_stack((p[j], p[j][0])), axis=0))    
    p = np.concatenate(p)
    q = np.concatenate(q)
    # Calculate squared distance between edges
    l2 = np.sum(q ** 2, axis=1) 
    # Catch l2 == 0 (exception)
    same = l2 < 1e-8 #if the distance is less than 1e-8, then it's considered the same vertex
    l2[same] = 1.
    # Calc t
    t = np.sum((np.array([gseast, gsnorth]) - p) * q, axis=1) / l2
    # Speed of boolean indices only slightly faster (negligible)
    # t must be limited between 0 and 1
    t = np.clip(t, 0., 1.)
    t[same] = 0.
    # Calculate closest point to each edge
    x1 = p[:,0] + t * q[:,0]
    y1 = p[:,1] + t * q[:,1]
    # Get distance squared
    d2 = (x1 - gseast) ** 2 + (y1 - gsnorth) ** 2
    # Sort distance
    ind = np.argsort(d2)
    x1  = x1[ind] #sort x1 from the smallest detour to the biggest 
    y1  = y1[ind]

    #In the end, the "shortest way out" solution is selected
    return x1, y1
    
def shortest_way_out_solutions(ARV, gseast, gsnorth,i):
    
    # It's just linalg, however credits to: http://stackoverflow.com/a/1501725
    
    # Loop through all exteriors and append. Afterwards concatenate
    p = []
    q = []
    for j in range(len(ARV)):
        p.append(np.array(ARV[j]))
        q.append(np.diff(np.row_stack((p[j], p[j][0])), axis=0))    
    p = np.concatenate(p)
    q = np.concatenate(q)
    # Calculate squared distance between edges
    l2 = np.sum(q ** 2, axis=1) 
    # Catch l2 == 0 (exception)
    same = l2 < 1e-8 #if the distance is less than 1e-8, then it's considered the same vertex
    l2[same] = 1.
    # Calc t
    t = np.sum((np.array([gseast, gsnorth]) - p) * q, axis=1) / l2
    # Speed of boolean indices only slightly faster (negligible)
    # t must be limited between 0 and 1
    t = np.clip(t, 0., 1.)
    t[same] = 0.
    # Calculate closest point to each edge
    x1 = p[:,0] + t * q[:,0]
    y1 = p[:,1] + t * q[:,1]
    # Get distance squared
    ##d2 = (x1 - gseast) ** 2 + (y1 - gsnorth) ** 2
    rel_angle = np.abs(np.arctan2(gsnorth, gseast) - np.arctan2(y1, x1))
    if rel_angle > 180:
        rel_angle  = np.abs(rel_angle - 360)
    # Sort distance
    ##ind = np.argsort(d2)
    # Sort relative angles
    ind = np.argsort(rel_angle)
    #Sort relative angle
    x1  = x1[ind] #sort x1 from the smallest detour to the biggest 
    y1  = y1[ind]

    #In the end, the "shortest way out" solution is selected
    return x1, y1