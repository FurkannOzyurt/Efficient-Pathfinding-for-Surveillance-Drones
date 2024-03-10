import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.awt.*;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;
import java.util.stream.Collectors;

public class IMECEPathFinder{
	  public int[][] grid;
	  public int height, width;
	  public int maxFlyingHeight;
	  public double fuelCostPerUnit, climbingCostPerUnit;
	  public double[][] cost;
	  public Map<int[],int[]> hash_map = new HashMap<>();

	  public IMECEPathFinder(String filename, int rows, int cols, int maxFlyingHeight, double fuelCostPerUnit, double climbingCostPerUnit){

		  grid = new int[rows][cols];
		  this.height = rows;
		  this.width = cols;
		  this.maxFlyingHeight = maxFlyingHeight;
		  this.fuelCostPerUnit = fuelCostPerUnit;
		  this.climbingCostPerUnit = climbingCostPerUnit;
		  this.cost = new double[height][width];

		  List<String> lines;
		  try {
			  lines = Files.readAllLines(Path.of(filename));
		  } catch (IOException e) {
			  throw new RuntimeException(e);
		  }

		  int min_value = Integer.MIN_VALUE;
		  int max_value = Integer.MAX_VALUE;

		  grid = new int[rows][cols];
		  for (int y = 0; y < rows; y++) {
			  String line = lines.get(y);
			  String[] values = line.trim().split("\\s+");
			  for (int x = 0; x < cols; x++) {
				  grid[y][x] = Integer.parseInt(values[x]);
			  }
		  }
	  }


	  /**
	   * Draws the grid using the given Graphics object.
	   * Colors should be grayscale values 0-255, scaled based on min/max elevation values in the grid
	   */
	  public void drawGrayscaleMap(Graphics g){

		  // TODO: draw the grid, delete the sample drawing with random color values given below
		  int min_value = Integer.MAX_VALUE;
		  int max_value = Integer.MIN_VALUE;
		  for (int[] row : grid) {
			  for (int col : row) {
				  min_value = Math.min(min_value, col);
				  max_value = Math.max(max_value, col);
			  }
		  }

		  int range = max_value - min_value;
		  for (int i = 0; i < grid.length; i++)
		  {
			  for (int j = 0; j < grid[0].length; j++) {
				  int value = grid[i][j];
				  int grayscaleValue = (int) ((value - min_value) / (double) range * 255);
				  g.setColor(new Color(grayscaleValue, grayscaleValue, grayscaleValue));
				  g.fillRect(j, i, 1, 1);
			  }
		  }
	  }

	public void createGrayscaleMap() {
		int min_value = Integer.MAX_VALUE;
		int max_value = Integer.MIN_VALUE;

		for (int[] row : grid) {
			for (int col : row) {
				min_value = Math.min(min_value, col);
				max_value = Math.max(max_value, col);
			}
		}

		int range = max_value - min_value;
		try (BufferedWriter writer = new BufferedWriter(new FileWriter("grayscaleMap.dat"))) {
			for (int i = 0; i < grid.length; i++) {
				for (int j = 0; j < grid[0].length; j++) {
					int value = grid[i][j];
					int grayscaleValue = (int) ((value - min_value) / (double) range * 255);
					writer.write(grayscaleValue + " ");
				}
				writer.newLine();
			}

		} catch (IOException e) {
			System.err.println("Dosya oluşturma hatası: " + e.getMessage());
		}
	}

	/**
	 * Get the most cost-efficient path from the source Point start to the destination Point end
	 * using Dijkstra's algorithm on pixels.
	 * @return the List of Points on the most cost-efficient path from start to end
	 */
	public List<Point> getMostEfficientPath(Point start, Point end) {

		List<Point> path = new ArrayList<>();

		// TODO: Your code goes here
		// TODO: Implement the Mission 0 algorithm here

		// Initialize distance and visited arrays
		boolean[][] prev_matrix = new boolean[height][width];
		for (int i = 0; i < height; i++) {
			Arrays.fill(cost[i], Double.POSITIVE_INFINITY);
		}

		// Priority queue for Dijkstra's algorithm
		PriorityQueue<Point> queue = new PriorityQueue<>(Comparator.comparingDouble(p -> cost[p.y][p.x]));
		cost[start.y][start.x] = 0;
		queue.add(start);

		while (!queue.isEmpty()) {
			Point current = queue.poll();
			prev_matrix[current.y][current.x] = false;

			if (current.x == end.x && current.y == end.y) {
				Point temp = end;
				while (temp != null) {
					path.add(0,current);
					temp.x = temp.prev_x;
					temp.y = temp.prev_y;
				}
				break; // Reached the destination, stop the algorithm
			}

			// Generate all possible neighbors
			List<Point> neighbors = generateNeighbors(current);

			for (Point neighbor : neighbors) {
				// Update distance if a shorter path is found
				if (cost[current.y][current.x] + calculateCost(current, neighbor) < cost[neighbor.y][neighbor.x]) {
					cost[neighbor.y][neighbor.x] = cost[current.y][current.x] + calculateCost(current, neighbor);
					neighbor.prev_x = current.x;
					neighbor.prev_y = current.y;
					queue.add(neighbor);
				}
			}
		}

		return path;
	}

	public double calculateCost(Point start, Point end) {
		double heightImpact;
		if(grid[start.y][start.x] >= grid[end.y][end.x]){
			heightImpact = 0;
		}
		else{
			heightImpact = grid[end.y][end.x] - grid[start.y][start.x];
		}
		return (calculateDistance(start, end) * fuelCostPerUnit) + (climbingCostPerUnit * heightImpact);
	}
	public double calculateDistance(Point start, Point end) {
		int x_dif = Math.abs(end.x - start.x);
		int y_dif = Math.abs(end.y - start.y);
		return Math.sqrt((x_dif * x_dif) + (y_dif * y_dif));
	}
	public boolean isValidPoint(int y, int x) {
		return x >= 0 && x < width && y >= 0 && y < height && grid[y][x] <= maxFlyingHeight;
	}

	public List<Point> generateNeighbors(Point point) {
		List<Point> neighbors = new ArrayList<>();

		int[][] directions = {
				{-1, 0},  // West, (x-1)
				{1, 0},   // East, (x+1)
				{0, -1},  // North, (y-1)
				{0, 1},   // South, (y+1)
				{-1, 1},  // South West, (x-1, y+1)
				{-1, -1}, // North West, (x-1, y-1)
				{1, 1},   // South East, (x+1, y+1)
				{1, -1}   // North East, (x+1, y-1)
		};

		for (int[] dir : directions) {
			int nx = point.x + dir[0];
			int ny = point.y + dir[1];
			if (isValidPoint(ny,nx)){
				neighbors.add(new Point(nx, ny));
			}
		}

		return neighbors;
	}

	/**
	 * Calculate the most cost-efficient path from source to destination.
	 * @return the total cost of this most cost-efficient path when traveling from source to destination
	 */
	public double getMostEfficientPathCost(List<Point> path){
		double totalCost = 0.0;

		// TODO: Your code goes here, use the output from the getMostEfficientPath() method
		totalCost = cost[path.get(path.size()-1).y][path.get(path.size()-1).x];
		System.out.println(path.get(path.size()-1));

		return totalCost;
	}


	/**
	 * Draw the most cost-efficient path on top of the grayscale map from source to destination.
	 */
	public void drawMostEfficientPath(Graphics g, List<Point> path){
		// TODO: Your code goes here, use the output from the getMostEfficientPath() method
		drawGrayscaleMap(g);
		g.setColor(Color.GREEN);

		for (Point point : path) {
			g.fillRect(point.x, point.y, 1, 1);
		}

	}

	/**
	 * Find an escape path from source towards East such that it has the lowest elevation change.
	 * Choose a forward step out of 3 possible forward locations, using greedy method described in the assignment instructions.
	 * @return the list of Points on the path
	 */
	public List<Point> getLowestElevationEscapePath(Point start){
		List<Point> pathPointsList = new ArrayList<>();

		// TODO: Your code goes here
		// TODO: Implement the Mission 1 greedy approach here
		pathPointsList.add(start);

		Point current = start;

		while (current.x < width - 1) {
			Point next = findNextStep(current);
			pathPointsList.add(next);
			current = next;
		}

		return pathPointsList;
	}

	public Point findNextStep(Point current) {
		int x = current.x + 1;
		int y = current.y;
		int minElevation_dif = Integer.MAX_VALUE;
		Point next = null;

		// Check the three adjacent cells in the next column
		for (int y_dif = -1; y_dif <= 1; y_dif++) {
			int y_next = y + y_dif;
			if (y_next >= 0 && y_next < height) {
				int elevation_dif = Math.abs(grid[y_next][x] - grid[current.y][current.x]);
				if (elevation_dif < minElevation_dif) {
					minElevation_dif = elevation_dif;
					next = new Point(x, y_next);
				} else if (elevation_dif == minElevation_dif) {
					if (y_dif == 0 && next != null) {
						next = new Point(x, y_next);
					}
					else if (y_dif < 0 && next != null && next.y < y) {
						next = new Point(x, y_next);
					}
				}
			}
		}

		return next;
	}

	/**
	 * Calculate the escape path from source towards East such that it has the lowest elevation change.
	 * @return the total change in elevation for the entire path
	 */
	public int getLowestElevationEscapePathCost(List<Point> pathPointsList){
		int totalChange = 0;

		// TODO: Your code goes here, use the output from the getLowestElevationEscapePath() method
		for (int i = 0; i < pathPointsList.size() - 1; i++) {
			Point current = pathPointsList.get(i);
			Point next = pathPointsList.get(i + 1);
			int elevation_dif = Math.abs(grid[next.y][next.x] - grid[current.y][current.x]);
			totalChange += elevation_dif;
		}

		return totalChange;
	}


	/**
	 * Draw the escape path from source towards East on top of the grayscale map such that it has the lowest elevation change.
	 */
	public void drawLowestElevationEscapePath(Graphics g, List<Point> pathPointsList){
		// TODO: Your code goes here, use the output from the getLowestElevationEscapePath() method
		drawGrayscaleMap(g);
		g.setColor(Color.RED);

		for (Point point : pathPointsList) {
			g.fillRect(point.x, point.y, 1, 1);
		}
	}


}
