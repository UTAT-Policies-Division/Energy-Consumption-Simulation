import enlib as el
import geolib as gl
import os.path

def load_decomposed_network(
        place_name="University of Toronto", 
        target_crs_epsg="EPSG:3348", 
        boundary_buffer_length=500, 
        simplification_tolerance=1):
    
    file_name = f"pickles/pickle_{place_name}_{target_crs_epsg}_{boundary_buffer_length}_{simplification_tolerance}.pkl"
    
    check_file = os.path.isfile(file_name)

    if check_file:
        print("Loading from Save...")
        return el.EnergyHelper.load(file_name)
    else:
        print("Generating Network...")
        nodes, edges, dedges, UID_to_ind, ind_to_UID = gl.get_decomposed_network(place_name, 
                                                                        target_crs_epsg, 
                                                                        boundary_buffer_length,
                                                                        simplification_tolerance)
        eh = el.EnergyHelper(nodes, edges, dedges, UID_to_ind, ind_to_UID,
                            10**(-2), gen_plot_data=True, demand=[])
        eh.save(file_name)
        return eh