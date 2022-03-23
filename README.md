## A* Path Planning for mobile robot
1. Download the file and extract it. Open terminal from this folder.
2. Run python3 akhilv_vishaal.py from the terminal (make sure the script is executable). Or Copy code to an IDE then run it. 
3. If the script is not executable, from the script directory open new terminal and run ```chmod +x akhilv_vishaal.py```
4. Enter the Start, Goal x, y coordinates and theta 
5. > Input coordinates are referenced such that the origin is at the top-left corner
6. Visualization is done after the map is explored and path is found.
7. The videos show the path generated for:
	1. Starting Values = (250, 150, 0)
	   Goal Values = (115, 45, 60)
	   Step size = 8
     
	2. Start coordinates = (30, 130, 30)
	   Goal coordinates = (120, 180, 0)
	   step size = 6


Note: Plotting the exploration may take some time depending on start and end positions.

Note: Please take into consideration the clearance allotted for the map. ie, x>15 y>15.  (x,y) = (10,10) will give an error.
