# Initialization Plans:

## Assumptions: 

1. Map network is undirected, since OSM doesn’t provide us with this data. 

2. Drones can carry any one single package from our system with payload less than or equal to MAX_PAYLOAD. 

3. Drone surface area integral is approximated has a singular area (temporary).
 

## How: 

1. Get a map of the region and convert it to a graph with stored lengths, for drone and truck each. 

    * Use Open Source Maps Library on Python to get map data, which is in the desired format of nodes and edges. Every edge has a length parameter, representing the distance between neighboring vertices. 

    * For trucks, we use the graph of vertices and edges which are “drivable”, and for drones, we use the graph of vertices and edges which are “walkable”. Note that every “drivable” edge must also be in the “walkable” graph, thus drone graph $G_d$ only has more vertices than the truck graph $G_t$ . 

2. Select source vertex and initialize demand over the graph. 

    * Select some vertex from $G_t$ near the “corner” of the geographical region we are considering, preferably a constant source which is the only way to go to a “depot”. 

    * Initializing demand: 

        * Use OSM to get all buildings in the target region. Sample `NUM_BUILDINGS` buildings randomly using `BUILDINGS_SELECTION_DISTRIBUTION`. 

        * For each selected building, define building demand ($db$) as `MAX_DEMAND_PER_BUILDING * X`, where $X$ is a random variable from 0 to 1, with distribution `DEMAND_PER_BUILDING_DISTRIBUTION`.  

        * Add vertex $v_b$ to $G_d$ and $G_t$, for each selected building (and add label “DESTINATION”). Store all such $v_b$ in a collection $D$. For both graphs, define set $S_v$ as the set of vertices which are closest to $v_b$ and not connected by an edge already. For each $v$ in $S_v$ , add edge ($v_b$, $v$) with length parameter direct length between the two vertices. 

3. Initialize loaded / unloaded - truck / drone weight function. 

    * We know the Epm function for drone given an edge depends on the following factors:

        * A. __Altitude__

            * Always less than or equal to `MAX_ALTITUDE` provided by `REGION_POLICY` object. May be fixed to minimum of `OPTIMAL_ALTITUDE` and `MAX_ALTITUDE`. 

        * B. __Payload__

            * Provided as a parameter. May be fixed if package weights are fixed. Note that the volumetric space of packages affects `NUM_PACKAGES`, not payload, based on assumption 2. 

        * C. __Velocity__

            * Always less than or equal to `MAX_VELOCITY`. Would likely be fixed to minimum of `OPTIMAL_VELOCITY` and `MAX_VELOCITY`. 

        * D. __Source to Destination Path__

            * The exact path will provide information on extra consumptions due changes required in speed during liftoff, landing and corners. Could be approximated and set as a constant energy overhead. 

        * E. __Noise__

            * Extra consumptions in energy due weather conditions, may need precise source to destination path, or can be approximated as a constant energy overhead, which can be made better by making different constants for different regions. 

    * We will have one function for each of the four possibilities: loaded truck, unloaded truck, loaded drone, unloaded drone. These functions are utilized in decision making for finding optimal paths and truck-drone switch points. 

4. Incorporate no-fly zones 

    * Let all no-fly zones be convex polygons on the geographical region of consideration. For all edges ($u$, $v$) of $G_d$ , if $u$ or $v$ lies in any one of the polygons, then remove this edge from $G_d$. If $u$ or $v$ become isolated, remove the vertex from $G_d$ as well. 
 
