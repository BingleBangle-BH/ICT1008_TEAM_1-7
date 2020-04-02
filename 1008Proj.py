from flask import Flask, request, render_template
from geopy.geocoders import Nominatim
from .ASTARTWalk import A_Star_Walk
from .walk_bus import walk_bus_algor
from .lrt_bus_walk import get_lrt_route, distance
import networkx as nx
import osmnx as ox
import folium
app = Flask(__name__)

#values control the lines to draw, if u need to draw more lines just store into another variable, drawing done below
bus = [(1.4044948, 103.9028788), (1.4043871, 103.9030588), (1.4032795, 103.9026913), (1.4031189, 103.902983), (1.4041337, 103.9036107),
          (1.4043804, 103.9037633), (1.4044809, 103.9038275), (1.4047209, 103.9039627), (1.4070332, 103.905242), (1.4072315, 103.9053287),
          (1.4073682, 103.9053812), (1.407324, 103.9054709), (1.4071493, 103.9058431), (1.4062428, 103.9073543), (1.4059892, 103.9077163),
          (1.4059216, 103.9078136), (1.4057384, 103.9080643), (1.4055475, 103.9083362), (1.4046704, 103.9095852), (1.4043211, 103.9100828),
          (1.4038478, 103.9107273), (1.4036731, 103.9109694), (1.4036067, 103.9110425), (1.4034314, 103.9112883), (1.4022109, 103.9129992)]

walking = []

lrt = [[(1.4039372, 103.9017148), (1.4043864, 103.9019645), (1.4048461, 103.902302), (1.4051499, 103.9024702), (1.4055467, 103.9026898),
        (1.405906, 103.9027954), (1.4070234, 103.9034045), (1.4072128, 103.9035684), (1.4074467, 103.9038372), (1.4074863, 103.9039291),
        (1.4075678, 103.904118), (1.4076142, 103.9044066), (1.4075884, 103.9046591), (1.4075086, 103.9049399), (1.4069728, 103.9060579),
        (1.4068067, 103.9063761), (1.406034, 103.9074971), (1.4055386, 103.9081329), (1.4053673, 103.9083654), (1.4050742, 103.9087633),
        (1.4046635, 103.9093817), (1.4034655, 103.911066), (1.4031732, 103.9114899), (1.4027366, 103.9120296), (1.4021767, 103.9128156)],
        [(1.4020923, 103.9127701)], [(1.4022109, 103.9129992), (1.4020712, 103.9131942), (1.4016311, 103.9138087), (1.401068, 103.9145951),
        (1.4010987, 103.9150204), (1.4015463, 103.9153359), (1.4026756, 103.9161375), (1.402906, 103.916156)], [(1.402906, 103.916156)]]

#sets up markers for bus stop, doesnt draw them here
busStops = [(1.404107, 103.9025242), (1.406245, 103.899574), (1.4053356, 103.8974167), (1.4024184, 103.8967454), (1.404107, 103.9025242)]

#start of the map, not super accurate, just for declaring
punggol = (1.403948, 103.909048)
#G = ox.graph_from_point(punggol, distance=1200, truncate_by_edge=True, network_type="walk")

# =========================================================
# PLACE IN INIT
# =========================================================

take_bus_distance = 150  # in meters. this value is for lrt+bus+walk
# if destination falls within this distance, user wont take a bus

punggol = (1.4053828, 103.9022239)  # punggol MRT station, change according to whatever coordinates you are using
G = ox.graph_from_point(punggol, distance=1200, truncate_by_edge=True, network_type="walk",infrastructure='way["highway"]')

G = ox.remove_isolated_nodes(G)

lrt_east = ox.graph_from_file(filename="data\lrt_pg_east.osm",
                              retain_all="true")  # retain all is essential
lrt_west = ox.graph_from_file(filename="data\lrt_pg_west.osm",
                              retain_all="true")

lrt_exits = ox.graph_from_file(filename="data\lrt_bridges.osm",
                              retain_all="true")

lrt_stations = nx.compose(lrt_east, lrt_west)

# =========================================================
# END OF "PLACE IN INIT" requirements
# =========================================================

@app.route("/")
@app.route("/home", methods=['GET', 'POST'])
def home():
    return render_template('home.html', textInput=my_form_post())

@app.route("/map")
def viewMap():
    return render_template('map.html')


def my_form_post():
    #stuff that runs the input check, if u just testing the line drawing, ignore this
    if request.method == 'POST':
        text = request.form.get('text')
        text1 = request.form.get('text1')
        pathTypeController = request.form.get('pathTypes')
        locator = Nominatim(user_agent="myGeocoder")
        location = locator.geocode(text, timeout=None)
        destination = locator.geocode(text1, timeout=None)
        locationXY = (location.longitude, location.latitude)
        destinationXY = (destination.longitude, destination.latitude)
        processed_text = ((location.latitude, location.longitude, destination.latitude, destination.longitude))
        m = folium.Map(location=[1.403948, 103.909048], zoom_start=15)

        #Map plotting
        #draws the lines, if u need more lines, duplicate this function, but change values into ur new variable name
        # for x in bus:
        #     folium.PolyLine(bus, color="red", weight=2.5, opacity=1).add_to(m)
        #
        if (pathTypeController == 'walk'):
            walking = A_Star_Walk(locationXY, destinationXY)
            for i in walking:
                folium.PolyLine(walking, color="blue", weight=2.5, opacity=1).add_to(m)

        elif (pathTypeController == 'walk_bus'):
            finalPath = walk_bus_algor(locationXY, destinationXY)
            for i in range(0, len(finalPath)):
                if i == 0:
                    folium.PolyLine(finalPath[i], color="blue", weight=2.5, opacity=1).add_to(m)
                elif i == 1:
                    folium.PolyLine(finalPath[i], color="red", weight=2.5, opacity=1).add_to(m)
                else:
                    folium.PolyLine(finalPath[i], color="blue", weight=2.5, opacity=1).add_to(m)


        elif (pathTypeController == 'walk_lrt'):
            finalPath = []

            lrtRoute = get_lrt_route(G, locationXY, destinationXY, lrt_stations, lrt_exits)
            lrtXY = lrtRoute[-1]
            lastLRT = lrtXY[::-1]  # flip lastLRT for proper XY formatting

            finalPath.append(lrtRoute)

            # call the walking function to get a route from last LRT station to destination
            finalPath.append(A_Star_Walk(lastLRT, destinationXY))

            for i in range(0, len(finalPath)):
                if i == 0:
                    folium.PolyLine(finalPath[i], color="blue", weight=2.5, opacity=1).add_to(m)
                elif i == 1:
                    folium.PolyLine(finalPath[i], color="red", weight=2.5, opacity=1).add_to(m)
                else:
                    folium.PolyLine(finalPath[i], color="blue", weight=2.5, opacity=1).add_to(m)

        elif (pathTypeController == 'walk_bus_lrt'):
            lrtPath = get_lrt_route(G, locationXY, destinationXY, lrt_stations, lrt_exits)
            lrtXY = lrtPath[-1]

            # map the lrt route first
            folium.PolyLine(lrtPath, color="blue", weight=2.5, opacity=1).add_to(m)

            # run walk_bus on the final LRT station
            flippedlrtXY = lrtXY[::-1]  # get XY of the last station

            # determine if there is a need to take a bus.
            print("distance between the two is: ", distance(flippedlrtXY, destinationXY))

            finalPath = []

            if distance(flippedlrtXY, destinationXY) > take_bus_distance:
                # destination is still 'far'. Route for walk+bus.
                finalPath = walk_bus_algor(flippedlrtXY, destinationXY)
            else:
                # destination is close enough to walk to. Route for walk.
                finalPath.append(A_Star_Walk(flippedlrtXY, destinationXY))

            #processed_text = flippedlrtXY
            for i in range(0, len(finalPath)):
                if i == 0:
                    folium.PolyLine(finalPath[i], color="blue", weight=2.5, opacity=1).add_to(m)
                elif i == 1:
                    folium.PolyLine(finalPath[i], color="red", weight=2.5, opacity=1).add_to(m)
                else:
                    folium.PolyLine(finalPath[i], color="blue", weight=2.5, opacity=1).add_to(m)

            # for i in range(0, len(finalPath)):
            #     if i == 0:
            #         folium.PolyLine(finalPath[i], color="blue", weight=2.5, opacity=1).add_to(m)

                # elif i == 1:
                #     folium.PolyLine(finalPath[i], color="red", weight=2.5, opacity=1).add_to(m)
                # elif i == 2:
                #     folium.PolyLine(finalPath[i], color="green", weight=2.5, opacity=1).add_to(m)
                # elif i == 3:
                #     folium.PolyLine(finalPath[i], color="purple", weight=2.5, opacity=1).add_to(m)
                # else:
                #     folium.PolyLine(finalPath[i], color="yellow", weight=2.5, opacity=1).add_to(m)

        # #puts bus stop markers
        # for y in busStops:
        #    folium.Marker(y, icon=folium.Icon(color='red', icon='bus', prefix='fa')).add_to(m)


        #Commented out version takes input, non-commented version takes the manual start and end from values
        folium.Marker([location.latitude, location.longitude], popup='<i>Start</i>').add_to(m)
        folium.Marker([destination.latitude, destination.longitude], popup='<i>Destination</i>').add_to(m)
        # folium.Marker([1.4044948, 103.9028788], popup='<i>Start</i>').add_to(m)
        # folium.Marker([1.4022109, 103.9129992], popup='<i>Destination</i>').add_to(m)

        m.save('templates/map.html')
        return processed_text, locationXY
    else:
        m = folium.Map(location=[1.403948, 103.909048], zoom_start=15)
        m.save('templates/map.html')
        return 0

