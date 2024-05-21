import com.cyberbotics.webots.controller.*;
import java.util.*;
import java.lang.Math;


public class CarController {

  // Variables
  Robot robot;
  Motor[] motors = new Motor[4];
  GPS gps;
  InertialUnit imu;
  DistanceSensor[] ds = new DistanceSensor[2];
  String[] motor_names = {"FR_motor", "FL_motor", "BR_motor", "BL_motor"};
  Node home = new Node(44, "warehouse", 80, 10);
  Node destination = new Node();
  Node[] map = new Node[45];
  int time_step = -1;
  double movement_velocity = 2;
  double turn_velocity = 30;
  int current_time = 0;
  double angle_threshold = 0.05;
  double distance_threshold = 0.5;
  
  // Constructor
  public CarController() {
    robot = new Robot();
    ds[0] = robot.getDistanceSensor("L_sensor");
    ds[1] = robot.getDistanceSensor("R_sensor");
    gps = robot.getGPS("GPS");
    imu = robot.getInertialUnit("IMU");
    
    for(int i = 0; i < motor_names.length; i++) {
      motors[i] = robot.getMotor(motor_names[i]);
    }
    
    for(int i = 0; i < motors.length; i++) {
      motors[i].setPosition(Double.POSITIVE_INFINITY);
      motors[i].setVelocity(0);
    }
    
    time_step = (int) Math.round(robot.getBasicTimeStep());
    
    ds[0].enable(time_step);
    ds[1].enable(time_step);
    gps.enable(time_step);
    imu.enable(time_step);
    
    // Corner GPS coordinates
    map[0] = new Node(0, "", 60, 10);
    map[1] = new Node(1, "", 42, 10);
    map[2] = new Node(2, "", 51, 4);
    map[3] = new Node(3, "", 51, 19);
    map[4] = new Node(4, "", 47, 71);
    map[5] = new Node(5, "", -66, 71);
    map[6] = new Node(6, "", -69, 30);
    map[7] = new Node(7, "", 15, 27);
    map[8] = new Node(8, "", 21, 13);
    map[9] = new Node(9, "", -69, -20);
    map[10] = new Node(10, "", 49, -18);
    map[11] = new Node(11, "", 54, -40);
    map[12] = new Node(12, "", 78, -45);
    map[13] = new Node(13, "", 78, -70);
    map[14] = new Node(14, "", -66, -70);
    
    // Building GPS coordinates
    map[15] = new Node(15, "motel", 51, -15); // motel reception
    map[16] = new Node(16, "church", 78, -45); // church
    map[17] = new Node(17, "suburban", 78, -70); // suburban house
    map[18] = new Node(18, "manor", 63, -73); // small manor
    map[19] = new Node(19, "modern", 54, -73); // modern house
    map[20] = new Node(20, "small_residential", 38, -73); // small residential building
    map[21] = new Node(21, "simple", 28, -73); // simple two-storey house
    map[22] = new Node(22, "medium_residential", 12, -73); // medium residential building
    map[23] = new Node(23, "modern_suburban", -5, -73); // modern suburban house
    map[24] = new Node(24, "big_residential", -23, -73); // big residential building
    map[25] = new Node(25, "garage", -38, -73); // house with garage
    map[26] = new Node(26, "bungalow", -66, -70.5); // bungalow style house
    map[27] = new Node(27, "composed", -69, -58); // composed house
    map[28] = new Node(28, "auditorium", -69, 5); // auditorium
    map[29] = new Node(29, "barn", -69, 22); // barn
    map[30] = new Node(30, "hollow", -69, 50); // hollow building
    map[31] = new Node(31, "old_residential", -7, 30); // old residential building
    map[32] = new Node(32, "hotel", 35, 10); // hotel
    map[33] = new Node(33, "restaurant", 35, -18); // fast food restaurant
    map[34] = new Node(34, "glass", 10, -19); // big glass tower
    map[35] = new Node(35, "gas_station", 51, 40); // gas station
    map[36] = new Node(36, "carwash", 51, 64); // carwash
    map[37] = new Node(37, "cyberbotics", 48, 70); // cyberbotics tower
    map[38] = new Node(38, "small_tower", 41, 73); // small residential tower
    map[39] = new Node(39, "commercial", 30, 73); // commercial building
    map[40] = new Node(40, "large_tower", 15, 73); // large residential tower
    map[41] = new Node(41, "building", -2, 73); // building
    map[42] = new Node(42, "construction", -15, 73); // building under construction
    map[43] = new Node(43, "museum", -50, 73); // museum
    map[44] = home;
    
    // Set adjacent points
    map[0].setAdjacent(new int[]{2, 3, 44});
    map[1].setAdjacent(new int[]{2, 3, 8, 32});
    map[2].setAdjacent(new int[]{0, 1, 10, 15});
    map[3].setAdjacent(new int[]{0, 1, 4, 35, 36});
    map[4].setAdjacent(new int[]{3, 5, 35, 36, 37, 38, 39, 40, 41, 42, 43});
    map[5].setAdjacent(new int[]{4, 6, 30, 38, 39, 40, 41, 42, 43});
    map[6].setAdjacent(new int[]{5, 7, 9, 28, 29, 30, 31});
    map[7].setAdjacent(new int[]{6, 8, 31});
    map[8].setAdjacent(new int[]{7, 1, 32});
    map[9].setAdjacent(new int[]{6, 10, 14, 27, 28, 29, 33, 34});
    map[10].setAdjacent(new int[]{2, 9, 11, 15, 33, 34});
    map[11].setAdjacent(new int[]{10, 12});
    map[12].setAdjacent(new int[]{11, 13, 16, 18});
    map[13].setAdjacent(new int[]{12, 14, 17, 18, 19, 20, 21, 22, 23, 24, 25});
    map[14].setAdjacent(new int[]{9, 13, 19, 20, 21, 22, 23, 24, 25, 26, 27});
    map[15].setAdjacent(new int[]{2, 10});
    map[16].setAdjacent(new int[]{12});
    map[17].setAdjacent(new int[]{13});
    map[18].setAdjacent(new int[]{12, 13});
    map[19].setAdjacent(new int[]{13, 14});
    map[20].setAdjacent(new int[]{13, 14});
    map[21].setAdjacent(new int[]{13, 14});
    map[22].setAdjacent(new int[]{13, 14});
    map[23].setAdjacent(new int[]{13, 14});
    map[24].setAdjacent(new int[]{13, 14});
    map[25].setAdjacent(new int[]{13, 14});
    map[26].setAdjacent(new int[]{14});
    map[27].setAdjacent(new int[]{9, 14});
    map[28].setAdjacent(new int[]{6, 9});
    map[29].setAdjacent(new int[]{6, 9});
    map[30].setAdjacent(new int[]{5, 6});
    map[31].setAdjacent(new int[]{6, 7});
    map[32].setAdjacent(new int[]{1, 8});
    map[33].setAdjacent(new int[]{9, 10});
    map[34].setAdjacent(new int[]{9, 10});
    map[35].setAdjacent(new int[]{3, 4});
    map[36].setAdjacent(new int[]{3, 4});
    map[37].setAdjacent(new int[]{4});
    map[38].setAdjacent(new int[]{4, 5});
    map[39].setAdjacent(new int[]{4, 5});
    map[40].setAdjacent(new int[]{4, 5});
    map[41].setAdjacent(new int[]{4, 5});
    map[42].setAdjacent(new int[]{4, 5});
    map[43].setAdjacent(new int[]{4, 5});
    map[44].setAdjacent(new int[]{0});
  }
  
  // Main method
  public static void main(String[] args) {
    CarController controller = new CarController();    
    controller.ControlLoop(args[0]);
  }
  
  public void ControlLoop(String building) {    
    boolean valid = false;
    while(!valid){
      for(Node node : map){
        if(building.toLowerCase().equals(node.getName().toLowerCase())){
          destination = node;
          valid = true;
          break;
        }
      }
    }
    
    ArrayList<Node> path = findShortestPath(home, destination);
    for(Node node : path){
    }
    
    int currentIndex = 0;
    while (robot.step(time_step) != -1) {
      
      if(currentIndex+1 == path.size() && path.get(currentIndex).getName().toLowerCase().equals("warehouse")){
        stop(motors);
        double current_time_1 = robot.getTime();
        double current_time_2= robot.getTime();
        do {
          stop(motors);
          current_time_2 = robot.getTime();
          robot.step(1);
        } while(current_time_2 < (current_time_1 + 3));
        return;
      }
      else if(currentIndex+1 == path.size()) {
        double current_time_1 = robot.getTime();
        double current_time_2= robot.getTime();
        do {
          stop(motors);
          current_time_2 = robot.getTime();
          robot.step(1);
        } while(current_time_2 < (current_time_1 + 50));
        Collections.reverse(path);
        currentIndex = 0;
      }
    
      double remainingDistance = calculateDistance(new Node(-1, "", gps.getValues()[0], gps.getValues()[1]), path.get(currentIndex+1));
    
      if(remainingDistance < distance_threshold && currentIndex+1 < path.size()) {
        currentIndex++;
      }
      
      if(currentIndex+1 == path.size()) {
        continue;
      }
      
      if(ds[0].getValue() < 550) {
        turnRight(motors);
      }
      else if(ds[1].getValue() < 550) {
        turnLeft(motors);
      }
      else {
        double currentOrientation = -1* (Math.PI / 2) + imu.getRollPitchYaw()[2];
        double desiredOrientation = calculateDesiredOrientation(path.get(currentIndex+1));
        double angleDifference = desiredOrientation - currentOrientation;
        
        // Find angle to next point
        if (angleDifference > Math.PI) {
          angleDifference -= 2 * Math.PI;
        } else if (angleDifference < -Math.PI) {
          angleDifference += 2 * Math.PI;
        }

        // Adjust angle of car to next point
        if (Math.abs(angleDifference) > angle_threshold) {
          if (angleDifference < 0) {
            turnRight(motors);
          }
          else {
            turnLeft(motors);
          }
        }
        else {
          goForward(motors);
        }
      }
    }
  }
  
  // Movement methods
  private void goForward(Motor[] motors) {  
    for(int i = 0; i < motors.length; i++) {
      motors[i].setVelocity(movement_velocity);
    }
  }
  private void stop(Motor[] motors) {  
    for(int i = 0; i < motors.length; i++) {
      motors[i].setVelocity(0);
    }
  }
  private void turnLeft(Motor[] motors) {  
    for(int i = 0; i < motors.length; i++) {
      if(i % 2 == 0) {
        motors[i].setVelocity(turn_velocity);
      }
      else {
        motors[i].setVelocity(-1 * turn_velocity);
      }
    }
  }
  private void turnRight(Motor[] motors) {  
    for(int i = 0; i < motors.length; i++) {
      if(i % 2 == 1){
        motors[i].setVelocity(turn_velocity);
      }
      else {
        motors[i].setVelocity(-1 * turn_velocity);
      }
    }
  }
  
  // Dijkstra's algorithm implementation
  public ArrayList<Node> findShortestPath(Node start, Node destination) {

    HashMap<Node, Double> distances = new HashMap<>();
    for (Node node : map) {
        distances.put(node, Double.POSITIVE_INFINITY);
    }

    HashMap<Node, Node> previous = new HashMap<>();

    PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingDouble(distances::get));
    distances.put(start, 0.0);
    queue.add(start);

    while (!queue.isEmpty()) {
        Node current = queue.poll();

        // Found the destination, stop searching
        if (current == destination) {
            break; 
        }
        
        // Calculate new distance through current node
        for (Node neighbor : current.adjacent) {
            double distance = distances.get(current) + calculateDistance(current, neighbor);
            if (distance < distances.get(neighbor)) {
                distances.put(neighbor, distance);
                previous.put(neighbor, current);
                queue.add(neighbor);
            }
        }
    }

    // Reverse path
    ArrayList<Node> path = new ArrayList<>();
    Node current = destination;
    while (previous.containsKey(current)) {
        path.add(current);
        current = previous.get(current);
    }
    path.add(start);
    Collections.reverse(path);
    return path;
  }

  // Pythag implementation
  private double calculateDistance(Node node1, Node node2) {
    double dx = node2.position.getX() - node1.position.getX();
    double dy = node2.position.getY() - node1.position.getY();
    return Math.sqrt(dx * dx + dy * dy);
  }
  
  // Calculate desired orientation from current position to next node
  private double calculateDesiredOrientation(Node nextNode) {
    double dx = nextNode.getPosition().getX() - gps.getValues()[0];
    double dy = nextNode.getPosition().getY() - gps.getValues()[1];
    return Math.atan2(dy, dx);
  }
  
  // Position object
  private class Position {
    double x;
    double y;
    
    public Position() {}
    
    public Position(double x, double y) {
      this.x = x;
      this.y = y;
    }
    
    public double getX() {
      return this.x;
    }
    
    public double getY() {
      return this.y;
    }
    
    public void setX(double x) {
      this.x = x;
    }
    
    public void setY(double y) {
      this.y = y;
    }
  }
  
  // Nodes within path
  private class Node {
    int id;
    Position position;
    String name;
    ArrayList<Node> adjacent = new ArrayList<Node>();

    public Node() {}

    public Node(int id, String name,  double x, double y) {
      this.id = id;
      this.name = name;
      this.position = new Position(x, y);
    }
    
    public int getID() {
      return this.id;
    }
    
    public String getName() {
      return this.name;
    }
    
    public Position getPosition() {
      return position;
    }
    
    public void setAdjacent(int[] IDs) {
      for(int id : IDs){
        for(Node node : map) {
          if(id == node.getID()) {
            this.adjacent.add(node);
          }
        }
      }
    }    
  }
}
