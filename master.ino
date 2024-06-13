// Define the possible facing directions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
#define WHEEL_IR_THRESHOLD 500
#define OBSTACLE_IR_THRESHOLD 500 // Should be able to detect it from a intersection

// Motor 1 control pins
const int IN1 = 11; // IN1 pin for motor 1
const int IN2 = 10; // IN2 pin for motor 1

// Motor 2 control pins
const int IN3 = 9; // IN1 pin for motor 2 (renamed to IN3 for clarity)
const int IN4 = 8; // IN2 pin for motor 2 (renamed to IN4 for clarity)

// Infrared sensor pins
const int IR_LEFT = A1;  // IR sensor for the left wheel
const int IR_RIGHT = A2; // IR sensor for the right wheel
const int IR_FRONT = A3; // IR sensor for checking front obstacles

int currentDirection;
int nextDirection;
Node *path;

// Define the grid dimensions
const int rows = 11;
const int cols = 11;

// Define the grid
char grid[rows][cols] = {
    // 0    1    2    3    4    5    6    7    8    9   10
    {' ', ' ', 'E', ' ', 'F', ' ', 'G', ' ', 'H', ' ', ' '},  // 0
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 1
    {'1', '-', '+', '-', '+', '-', '+', '-', '+', '-', '5'},  // 2
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 3
    {'2', '-', '+', '-', '+', '-', '+', '-', '+', '-', '6'},  // 4
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 5
    {'3', '-', '+', '-', '+', '-', '+', '-', '+', '-', '7'},  // 6
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 7
    {'4', '-', '+', '-', '+', '-', '+', '-', '+', '-', '8'},  // 8
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 9
    {' ', ' ', 'A', ' ', 'B', ' ', 'C', ' ', 'D', ' ', ' '}}; // 10

// Function to find a character in the grid and return its coordinates
void findCharInGrid(char c, int &x, int &y)
{
    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            if (grid[i][j] == c)
            {
                x = i;
                y = j;
                return;
            }
        }
    }
}

struct Node
{
    int position[2];
    int g;
    Node *parent;
};

bool is_valid_move(int current_pos[2], int next_pos[2])
{
    int row = current_pos[0];
    int col = current_pos[1];
    int next_row = next_pos[0];
    int next_col = next_pos[1];

    if (next_row < 0 || next_row >= rows || next_col < 0 || next_col >= cols)
    {
        return false;
    }

    if (grid[next_row][next_col] == ' ' || grid[next_row][next_col] == 'x')
    { // Consider 'x' as obstacle
        return false;
    }

    if (row == next_row)
    { // Horizontal move
        if (next_col - col == 2 && grid[row][col + 1] != '-')
        {
            return false;
        }
        else if (next_col - col == -2 && grid[row][col - 1] != '-')
        {
            return false;
        }
    }
    else if (col == next_col)
    { // Vertical move
        if (next_row - row == 2 && grid[row + 1][col] != '|')
        {
            return false;
        }
        else if (next_row - row == -2 && grid[row - 1][col] != '|')
        {
            return false;
        }
    }

    return true;
}

Node *dijkstra(int start[2], int end[2])
{
    Node *start_node = new Node;
    start_node->position[0] = start[0];
    start_node->position[1] = start[1];
    start_node->g = 0;
    start_node->parent = NULL;

    Node *open_list[rows * cols];
    int open_list_size = 0;
    open_list[open_list_size++] = start_node;

    bool closed_list[rows][cols] = {false};

    while (open_list_size > 0)
    {
        Node *current_node = open_list[0];
        int current_index = 0;

        for (int i = 1; i < open_list_size; i++)
        {
            if (open_list[i]->g < current_node->g)
            {
                current_node = open_list[i];
                current_index = i;
            }
        }

        open_list_size--;
        for (int i = current_index; i < open_list_size; i++)
        {
            open_list[i] = open_list[i + 1];
        }

        closed_list[current_node->position[0]][current_node->position[1]] = true;

        if (current_node->position[0] == end[0] && current_node->position[1] == end[1])
        {
            return current_node;
        }

        int neighbors[4][2] = {
            {0, 2}, {2, 0}, {0, -2}, {-2, 0} // Possible moves: right, down, left, up
        };

        for (int i = 0; i < 4; i++)
        {
            int neighbor_pos[2] = {current_node->position[0] + neighbors[i][0], current_node->position[1] + neighbors[i][1]};

            if (is_valid_move(current_node->position, neighbor_pos) && !closed_list[neighbor_pos[0]][neighbor_pos[1]])
            {
                Node *neighbor_node = new Node;
                neighbor_node->position[0] = neighbor_pos[0];
                neighbor_node->position[1] = neighbor_pos[1];
                neighbor_node->g = current_node->g + 1;
                neighbor_node->parent = current_node;

                bool skip = false;
                for (int j = 0; j < open_list_size; j++)
                {
                    if (open_list[j]->position[0] == neighbor_node->position[0] && open_list[j]->position[1] == neighbor_node->position[1] && open_list[j]->g <= neighbor_node->g)
                    {
                        skip = true;
                        break;
                    }
                }

                if (!skip)
                {
                    open_list[open_list_size++] = neighbor_node;
                }
                else{
                    delete neighbor_node;
                }
            }
        }
    }
    return NULL;
}

// Function to parse the input and find the start and end coordinates
void parseInput(char startChar, char endChar, int &startX, int &startY, int &endX, int &endY)
{
    findCharInGrid(startChar, startX, startY);
    findCharInGrid(endChar, endX, endY);
}

int getStartDirection(char startChar)
{
    if (startChar >= 'E' && startChar <= 'H')
    {
        return SOUTH;
    }
    else if (startChar >= '1' && startChar <= '4')
    {
        return EAST;
    }
    else if (startChar >= '5' && startChar <= '8')
    {
        return WEST;
    }
    else if (startChar >= 'A' && startChar <= 'D')
    {
        return NORTH;
    }
}

void printPath(struct Node *path)
{
    // Create a temporary copy of the grid for display purposes
    char tempGrid[rows][cols];
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            tempGrid[i][j] = grid[i][j];
        }
    }

    // Mark the path on the temporary grid
    Node *node = path;
    while (node != NULL)
    {
        tempGrid[node->position[0]][node->position[1]] = 'P';
        node = node->parent;
    }

    // Print the temporary grid with the path marked
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            Serial.print(tempGrid[i][j]);
        }
        Serial.println();
    }
}

void setup()
{
    Serial.begin(9600);
    // Motor control pin modes
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Example usage with dynamic obstacles
    char startChar = 'A';
    char endChar = 'H';
    int startX, startY, endX, endY;
    parseInput(startChar, endChar, startX, startY, endX, endY);

    path = dijkstra(new int[2]{startX, startY}, new int[2]{endX, endY});
    currentDirection = getStartDirection(startChar);
    nextDirection = currentDirection;
    printPath(path);
    Serial.print("Starting facing direction: ");
    switch (currentDirection)
    {
    case NORTH:
        Serial.println("NORTH");
        break;
    case EAST:
        Serial.println("EAST");
        break;
    case SOUTH:
        Serial.println("SOUTH");
        break;
    case WEST:
        Serial.println("WEST");
        break;
    }
}

void driveForward()
{
    int leftSensorValue = analogRead(IR_LEFT);
    int rightSensorValue = analogRead(IR_RIGHT);

    // If only the left sensor detects a line, turn slightly right to realign
    if (leftSensorValue < WHEEL_IR_THRESHOLD)
    {
        digitalWrite(IN1, HIGH); // Keep motor 1 running
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); // Slow down motor 2
        digitalWrite(IN4, LOW);
    }
    // If only the right sensor detects a line, turn slightly left to realign
    else if (rightSensorValue < WHEEL_IR_THRESHOLD)
    {
        digitalWrite(IN1, LOW); // Slow down motor 1
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); // Keep motor 2 running
        digitalWrite(IN4, LOW);
    }
    // If neither sensor detects a line, drive straight forward
    else
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
}

void turnLeft(bool isAtIntersection)
{
    bool rightSensorEnteredFirstLine = false;
    bool rightSensorExitedFirstLine = false;
    bool rightSensorEnteredSecondLine = false;
    bool rightSensorExitedSecondLine = false;

    while (!rightSensorExitedSecondLine)
    {
        // Read from the right sensor
        int rightSensorValue = analogRead(IR_RIGHT);

        // If not at an intersection, check if the left sensor has entered the first line
        if (isAtIntersection || !rightSensorEnteredFirstLine && rightSensorValue < WHEEL_IR_THRESHOLD)
        {
            rightSensorEnteredFirstLine = true;
        }

        // Check if the right sensor has exited the first line
        if (rightSensorEnteredFirstLine && !rightSensorExitedFirstLine && rightSensorValue > WHEEL_IR_THRESHOLD)
        {
            rightSensorExitedFirstLine = true;
        }

        // Check if the right sensor has entered the second line
        if (rightSensorExitedFirstLine && !rightSensorEnteredSecondLine && rightSensorValue < WHEEL_IR_THRESHOLD)
        {
            rightSensorEnteredSecondLine = true;
        }

        // Check if the right sensor has exited the second line
        if (rightSensorEnteredSecondLine && !rightSensorExitedSecondLine && rightSensorValue > WHEEL_IR_THRESHOLD)
        {
            rightSensorExitedSecondLine = true;
        }

        // Turn left
        digitalWrite(IN1, LOW); // Stop motor 1
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); // Keep motor 2 running
        digitalWrite(IN4, LOW);
    }
}

void turnRight(bool isAtIntersection)
{
    bool leftSensorEnteredFirstLine = false;
    bool leftSensorExitedFirstLine = false;
    bool leftSensorEnteredSecondLine = false;
    bool leftSensorExitedSecondLine = false;

    while (!leftSensorExitedSecondLine)
    {
        // Read from the left sensor
        int leftSensorValue = analogRead(IR_LEFT);

        // If not at an intersection, check if the left sensor has entered the first line
        if (isAtIntersection || !leftSensorEnteredFirstLine && leftSensorValue < WHEEL_IR_THRESHOLD)
        {
            leftSensorEnteredFirstLine = true;
        }

        // Check if the left sensor has exited the first line
        if (leftSensorEnteredFirstLine && !leftSensorExitedFirstLine && leftSensorValue > WHEEL_IR_THRESHOLD)
        {
            leftSensorExitedFirstLine = true;
        }

        // Check if the left sensor has entered the second line
        if (leftSensorExitedFirstLine && !leftSensorEnteredSecondLine && leftSensorValue < WHEEL_IR_THRESHOLD)
        {
            leftSensorEnteredSecondLine = true;
        }

        // Check if the left sensor has exited the second line
        if (leftSensorEnteredSecondLine && !leftSensorExitedSecondLine && leftSensorValue > WHEEL_IR_THRESHOLD)
        {
            leftSensorExitedSecondLine = true;
        }

        // Turn right
        digitalWrite(IN1, HIGH); // Keep motor 1 running
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); // Stop motor 2
        digitalWrite(IN4, LOW);
    }
}

bool isThereObstacle()
{
    int frontSensorValue = analogRead(IR_FRONT);
    return frontSensorValue < OBSTACLE_IR_THRESHOLD;
}

bool isThereIntersection()
{
    int leftSensorValue = analogRead(IR_LEFT);
    int rightSensorValue = analogRead(IR_RIGHT);
    return leftSensorValue < WHEEL_IR_THRESHOLD && rightSensorValue < WHEEL_IR_THRESHOLD;
}


void updateCurrentPosition()
{
    if (path == NULL || path->parent == NULL)
    {
        // If path is empty or only has one node, we cannot update the position.
        return;
    }

    // Find the second to last node and the last node
    Node *thirdToLast = path;
    Node *secondToLast = path->parent;
    Node *last = path->parent->parent;
    while (last->parent != NULL)
    {
        thirdToLast = secondToLast;
        secondToLast = last;
        last = last->parent;
    }

    if(last != NULL){
        delete last;
        secondToLast->parent = NULL;
    }
    else{
        delete secondToLast;
        thirdToLast->parent = NULL;
    }
    // Now, secondToLast is the new last node in the path
    // Determine the next direction based on the positions of the second to last node and the last node
    nextDirection = getNextDirection(thirdToLast, secondToLast);
    // Remove the last node
    printPath(path);
    Serial.println("Updated current position: ");
    Serial.print(secondToLast->position[0]);
    Serial.print(", ");
    Serial.println(secondToLast->position[1]);
    Serial.print("Next direction: ");
    printDirection(nextDirection);

}

int getNextDirection(Node* nextPosition, Node* currentPosition){
    int deltaX = nextPosition->position[1] - currentPosition->position[1];
    int deltaY = nextPosition->position[0] - currentPosition->position[0];

    if (deltaX == 2)
    {
        return EAST;
    }
    else if (deltaX == -2)
    {
        return WEST;
    }
    else if (deltaY == 2)
    {
        return SOUTH;
    }
    else if (deltaY == -2)
    {
        return NORTH;
    }
}

void turnToNewDirection(bool isAtIntersection)
{
    int directionDifference = nextDirection - currentDirection;

    if (directionDifference == 1 || directionDifference == -3)
    {
        turnRight(isAtIntersection);
    }
    else if (directionDifference == -1 || directionDifference == 3)
    {
        turnLeft(isAtIntersection);
    }

    currentDirection = nextDirection;
    Serial.print("Turned to new direction: ");
    printDirection(currentDirection);
}

void printDirection(int direction)
{
    switch (direction)
    {
    case NORTH:
        Serial.println("NORTH");
        break;
    case EAST:
        Serial.println("EAST");
        break;
    case SOUTH:
        Serial.println("SOUTH");
        break;
    case WEST:
        Serial.println("WEST");
        break;
    }
}

void handleObstacleAndRecalculateRoute()
{
    if (path == NULL || path->parent == NULL)
    {
        // Path is too short, cannot handle obstacle
        return;
    }

    Node *secondToLast = path;
    Node *last = path->parent;
    while (last->parent != NULL)
    {
        secondToLast = last;
        last = last->parent;
    }

    // Calculate the position between last and secondToLast nodes
    int obstacleX = (last->position[0] + secondToLast->position[0]) / 2;
    int obstacleY = (last->position[1] + secondToLast->position[1]) / 2;

    // Check if the position is valid for an obstacle ('-' or '|')
    if (grid[obstacleX][obstacleY] == '-' || grid[obstacleX][obstacleY] == '|')
    {
        // Mark the position as an obstacle
        grid[obstacleX][obstacleY] = 'x';

        // Recalculate the route from the current position to the destination
        int start[2] = {last->position[0], last->position[1]};
        int end[2] = {path->position[0], path->position[1]};
        // You might need to implement a way to retrieve or store the end position
        Node *newPath = dijkstra(start, end);

        // Update the path with the new route
        // Make sure to properly delete/free the old path to avoid memory leaks
        deletePath(path);
        path = newPath;
        Serial.println("Recalculated path:");
        printPath(path);

        // Optionally, print the new path or take other actions as needed
    }
    secondToLast = path;
    last = path->parent;

    {
        secondToLast = last;
        last = last->parent;
    }

    nextDirection = getNextDirection(secondToLast, last);
    Serial.print("Next direction: ");
    printDirection(nextDirection);
}

void deletePath(Node *node)
{
    if (node == NULL)
    {
        return;
    }

    deletePath(node->parent);
    delete node;
}

void loop()
{ 

    driveForward();
    if (isThereObstacle())
    {
        Serial.println("Obstacle detected!");
        handleObstacleAndRecalculateRoute();
        if(currentDirection != nextDirection){
            turnToNewDirection(false);
        }
    }
    else if (isThereIntersection())
    {
        updateCurrentPosition();
        if (currentDirection != nextDirection)
        {
            turnToNewDirection(true);
        }
        else{ 
            while(isThereIntersection()){
                driveForward();
            }
        }
    }
}

