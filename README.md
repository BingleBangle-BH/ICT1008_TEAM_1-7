# ICT1008 Python Project 

Contents | Built With 
---------|-------------
[Punggol Traveller](#punggol-traveller)|[Python 3.7](https://docs.python.org/3.7/) 
 [Installing Environment with .YML](#Installing-Environment-with-YML) | [Flask](https://flask.palletsprojects.com/en/1.1.x/)
 [Running Application](#running-application) | [Geopy](https://geopy.readthedocs.io/en/stable/)
[Project Details](#project-details) | [Folium](https://python-visualization.github.io/folium/) 
  [Collaborators with Contributions](#collaborators-with-contributions) | [Osmnx](https://osmnx.readthedocs.io/en/stable/)
  . | [LTADataMall](https://www.mytransport.sg/content/mytransport/home/dataMall/dynamic-data.html#Public%20Transport/)

## Punggol Traveller

**Punggol Traveller** is a web mapping service that offers an open street map and interactive route planning for traveling by foot, bus and trains. By using Punggol traveller, users will be able to get around the district quickly from one to another in the shortest time span. 

[Youtube Video](https://www.youtube.com/watch?v=hri77gZqfho) to install and run the application

**Internet connection is required for this application to run.**

**The application will take some time to start up due to the creations of multi-dimensional graphs.**

## Installing Environment with .YML

1. Download and install [Anaconda Navigator](https://www.anaconda.com/distribution/) for Python 3.7.

2. Open Anaconda Navigator and click on Environments

![Environment](https://i.ibb.co/mB0H9nz/step-2.png)

3. Click on import

![Import](https://i.ibb.co/LC5yj9G/step-3.png)

4. Name your environment and choose DSA_ICT1008_Libraries.yml (Located in setup) in specification file 

![Import](https://i.ibb.co/FY1qZ7H/step-4.png)

5. Upon doing so, anaconda will start installing the package environment. Select this newly created environment as your interpreter.

## Running Application

6. Open the terminal of the anaconda environment from Step 5.

7. Change the directory of the terminal to the folder where the file '1008Proj.py' is located.

![Import](https://i.ibb.co/pWLrmyH/1008-Report-3.png)

8. Enter the next three lines into the terminal:
	set FLASK_APP=1008Proj.py
	set FLASK_DEBUG=1
	flask run

![Import](https://i.ibb.co/cJ5zWLL/1008-Report-2.png)

9. The console will print out a URL, copy that url, adding home to the end of it, and run it in your browser.
	(e.g. http://128.0.0.1:5000/home)

![Import](https://i.ibb.co/ys7qLDt/1008-Report-1.png)
    
## Project Details

Implementing and plotting network
* [X] **Walk Network**
* [X] **Walk + Bus Network**
* [X] **Walk + LRT Network**
* [X] **Walk + LRT + Bus Network**


## Collaborators with Contributions
**TEAM 1-7** 

1. **Chin Bing Hong** | [@skyarebiue](https://github.com/skyarebiue) | Dijkstra and A* Algorithm, Bus + Walk
2. **Chang Lik Jack** | [@yolothedino](https://github.com/yolothedino) | Greedy Algorithm, LRT/MRT + Walk
3. **Bryan Sim** | [@BryanSim-SIT](https://github.com/BryanSim-SIT)  | GUI, Poster
4. **Miki Tan** | [@MTKL1106](https://github.com/MTKL1106) | Poster
