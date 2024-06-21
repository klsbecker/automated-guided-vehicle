/**
 * AGV (Automated Guided Vehicle) control system
 * Universidade do Vale do Rio dos Sinos - UNISINOS
 * Computer Engineering - Microprocessor Systems
 * 
 * Authors:
 * - Carlos Eduardo
 * - Klaus Becker
*/
#include <Arduino.h>
#include <Thread.h>

#define START_POSITION 'H'
#define END_POSITION   '3'

#define MOTOR_LEFT_FORWARD_PIN   6
#define MOTOR_LEFT_BACKWARD_PIN  7
#define MOTOR_RIGHT_FORWARD_PIN  4
#define MOTOR_RIGHT_BACKWARD_PIN 5

#define FRONT_IR_SENSOR_PIN 9
#define LEFT_INTERSECTION_IR_SENSOR_PIN  10
#define RIGHT_INTERSECTION_IR_SENSOR_PIN 8
#define LEFT_LF_IR_SENSOR_PIN 2
#define RIGHT_LF_IR_SENSOR_PIN 3

#define FRONT_IR_SENSOR_ACTIVE_STATE LOW
#define LEFT_IR_SENSOR_ACTIVE_STATE  HIGH
#define RIGHT_IR_SENSOR_ACTIVE_STATE HIGH

#define GRID_LAYOUT_SIZE 11

enum {
    NORTH_DIRECTION = 0,
    EAST_DIRECTION,
    SOUTH_DIRECTION,
    WEST_DIRECTION
};

enum {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
};

struct Node
{
    int position[2];
    int g;
    Node *parent;
};

static struct diags {
    bool leftSensorDetected;
    bool rightSensorDetected;
    bool frontSensorDetected;
    bool intersectionDetected;
    uint8_t currentDirection;
    uint8_t nextDirection;
} diags;

Node *path;

/**
 * Define the grid with the following characters:
 * - ' ' for empty spaces
 * - '|' for vertical walls
 * - '-' for horizontal walls
 * - '+' for intersections
 * - [1-8] and [A-H] for initial or destination positions
 *
 * Grid layout:
 *       E   F   G   H
 *       |   |   |   |
 *   1 - + - + - + - + - 5
 *       |   |   |   |
 *   2 - + - + - + - + - 6
 *       |   |   |   |
 *   3 - + - + - + - + - 7
 *       |   |   |   |
 *   4 - + - + - + - + - 8
 *       |   |   |   |
 *       A   B   C   D
 */
char grid[GRID_LAYOUT_SIZE][GRID_LAYOUT_SIZE] = {
    {' ', ' ', 'E', ' ', 'F', ' ', 'G', ' ', 'H', ' ', ' '}, 
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '}, 
    {'1', '-', '+', '-', '+', '-', '+', '-', '+', '-', '5'}, 
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '}, 
    {'2', '-', '+', '-', '+', '-', '+', '-', '+', '-', '6'}, 
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '}, 
    {'3', '-', '+', '-', '+', '-', '+', '-', '+', '-', '7'}, 
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '}, 
    {'4', '-', '+', '-', '+', '-', '+', '-', '+', '-', '8'}, 
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '}, 
    {' ', ' ', 'A', ' ', 'B', ' ', 'C', ' ', 'D', ' ', ' '}};

Thread diagsThread = Thread();

void printDiagnosticInfo(void) {
    Serial.println("");

    Serial.print("LEFT | RIGHT | FRONT | INTERSECTION | CURRENT | NEXT\n");
    Serial.print("-----------------------------------------------------\n");
    Serial.print(diags.leftSensorDetected ? "  X  | " : "     | ");
    Serial.print(diags.rightSensorDetected ? "  X   | " : "      | ");
    Serial.print(diags.frontSensorDetected ? "  X   | " : "      | ");
    Serial.print(diags.intersectionDetected ? "      X       | " : "              | ");
    printDirection(diags.currentDirection);
    Serial.print(" | ");
    printDirection(diags.nextDirection);
    
    Serial.println("");
    printPath(path);
}

int* getPositionCoordinates(char position) {
    static int coords[2] = {-1, -1};
    for (int i = 0; i < GRID_LAYOUT_SIZE; ++i) {
        for (int j = 0; j < GRID_LAYOUT_SIZE; ++j) {
            if (grid[i][j] == position) {
                coords[0] = i;
                coords[1] = j;
                return coords;
            }
        }
    }

    return coords;
}
bool isLeftLineFollowerSensorDetected(void) 
{   
    diags.leftSensorDetected = digitalRead(LEFT_LF_IR_SENSOR_PIN) == LEFT_IR_SENSOR_ACTIVE_STATE;
    return diags.leftSensorDetected;
}

bool isRightLineFollowerSensorDetected(void) 
{
    diags.rightSensorDetected = digitalRead(RIGHT_LF_IR_SENSOR_PIN) == RIGHT_IR_SENSOR_ACTIVE_STATE;
    return diags.rightSensorDetected;
}

bool isLeftIntersectionSensorDetected(void)
{
    return digitalRead(LEFT_INTERSECTION_IR_SENSOR_PIN);
}

bool isRightIntersectionSensorDetected(void)
{
    return digitalRead(RIGHT_INTERSECTION_IR_SENSOR_PIN);
}

bool isFrontSensorDetected(void) 
{
    diags.frontSensorDetected = digitalRead(FRONT_IR_SENSOR_PIN) == FRONT_IR_SENSOR_ACTIVE_STATE;
    return diags.frontSensorDetected;
}

bool isValidMove(int currentPos[2], int nextPos[2]) {
    int currentRow = currentPos[0];
    int currentCol = currentPos[1];
    int nextRow = nextPos[0];
    int nextCol = nextPos[1];

    // Check if next position is out of bounds
    if (nextRow < 0 || nextRow >= GRID_LAYOUT_SIZE || nextCol < 0 || nextCol >= GRID_LAYOUT_SIZE) {
        return false;
    }

    // Check if next position is an obstacle or empty space
    if (grid[nextRow][nextCol] == ' ' || grid[nextRow][nextCol] == 'x') {
        return false;
    }

    // Check for horizontal move
    if (currentRow == nextRow) {
        if (abs(nextCol - currentCol) == 2 && grid[currentRow][(currentCol + nextCol) / 2] != '-') {
            return false;
        }
    }
    // Check for vertical move
    else if (currentCol == nextCol) {
        if (abs(nextRow - currentRow) == 2 && grid[(currentRow + nextRow) / 2][currentCol] != '|') {
            return false;
        }
    }
    // If the move is not purely horizontal or vertical
    else {
        return false;
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

    Node *open_list[GRID_LAYOUT_SIZE * GRID_LAYOUT_SIZE];
    int open_list_size = 0;
    open_list[open_list_size++] = start_node;

    bool closed_list[GRID_LAYOUT_SIZE][GRID_LAYOUT_SIZE] = {false};

    while (open_list_size > 0)
    {
        Node *current_node = open_list[0];
        int current_index = 0;

        for (int i = 1; i < open_list_size; i++)
        {
            if (open_list[i]->g < current_node->g) {
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

        if (current_node->position[0] == end[0] && current_node->position[1] == end[1]) {
            return current_node;
        }

        int neighbors[4][2] = {
            {0, 2},  // Move right:  No change in the row, move two columns to the right
            {2, 0},  // Move down:   Move two rows down, no change in the column
            {0, -2}, // Move left:   No change in the row, move two columns to the left
            {-2, 0}  // Move up:     Move two rows up, no change in the column
        };

        for (int i = 0; i < 4; i++)
        {
            int neighbor_pos[2] = {current_node->position[0] + neighbors[i][0], current_node->position[1] + neighbors[i][1]};

            if (isValidMove(current_node->position, neighbor_pos) && !closed_list[neighbor_pos[0]][neighbor_pos[1]]) {
                Node *neighbor_node = new Node;
                neighbor_node->position[0] = neighbor_pos[0];
                neighbor_node->position[1] = neighbor_pos[1];
                neighbor_node->g = current_node->g + 1;
                neighbor_node->parent = current_node;

                bool skip = false;
                for (int j = 0; j < open_list_size; j++)
                {
                    if (open_list[j]->position[0] == neighbor_node->position[0] && open_list[j]->position[1] == neighbor_node->position[1] && open_list[j]->g <= neighbor_node->g) {
                        skip = true;
                        break;
                    }
                }

                if (!skip) {
                    open_list[open_list_size++] = neighbor_node;
                } else {
                    delete neighbor_node;
                }
            }
        }
    }
    return NULL;
}

int getStartDirection(char startChar)
{
    if (startChar >= 'E' && startChar <= 'H') {
        Serial.println("SOUTH");
        return SOUTH_DIRECTION;
    } else if (startChar >= '1' && startChar <= '4') {
        Serial.println("EAST");
        return EAST_DIRECTION;
    } else if (startChar >= '5' && startChar <= '8') {
        Serial.println("WEST");
        return WEST_DIRECTION;
    } else if (startChar >= 'A' && startChar <= 'D') {
        Serial.println("NORTH");
        return NORTH_DIRECTION;
    }
}

void printPath(struct Node *path)
{
    // Create a temporary copy of the grid for display purposes
    char tempGrid[GRID_LAYOUT_SIZE][GRID_LAYOUT_SIZE];
    for (int i = 0; i < GRID_LAYOUT_SIZE; i++)
    {
        for (int j = 0; j < GRID_LAYOUT_SIZE; j++)
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
    for (int i = 0; i < GRID_LAYOUT_SIZE; i++)
    {
        for (int j = 0; j < GRID_LAYOUT_SIZE; j++)
        {
            Serial.print(tempGrid[i][j]);
        }
        Serial.println();
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(MOTOR_LEFT_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD_PIN, OUTPUT);

    int startX, startY, endX, endY;
    parseInput(START_POSITION, END_POSITION, startX, startY, endX, endY);

    path = dijkstra(new int[2]{startX, startY}, new int[2]{endX, endY});
    // path = dijkstra(getPositionCoordinates(START_POSITION), getPositionCoordinates(END_POSITION));

    diags.currentDirection = getStartDirection(START_POSITION);
    diags.nextDirection = diags.currentDirection;

    diagsThread.onRun(printDiagnosticInfo);
    diagsThread.setInterval(200);
}

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

void parseInput(char startChar, char endChar, int &startX, int &startY, int &endX, int &endY)
{
    findCharInGrid(startChar, startX, startY);
    findCharInGrid(endChar, endX, endY);
}



void drive(uint8_t motorLeft, uint8_t motorRight)
{
    switch (motorLeft) {
        case MOTOR_STOP:
            digitalWrite(MOTOR_LEFT_FORWARD_PIN,  LOW);
            digitalWrite(MOTOR_LEFT_BACKWARD_PIN, LOW);
            break;
        case MOTOR_FORWARD:
            digitalWrite(MOTOR_LEFT_FORWARD_PIN,  HIGH);
            digitalWrite(MOTOR_LEFT_BACKWARD_PIN, LOW);
            break;
        case MOTOR_BACKWARD:
            digitalWrite(MOTOR_LEFT_FORWARD_PIN,  LOW);
            digitalWrite(MOTOR_LEFT_BACKWARD_PIN, HIGH);
            break;
    }

    switch (motorRight) {
        case MOTOR_STOP:
            digitalWrite(MOTOR_RIGHT_FORWARD_PIN,  LOW);
            digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, LOW);
            break;
        case MOTOR_FORWARD:
            digitalWrite(MOTOR_RIGHT_FORWARD_PIN,  HIGH);
            digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, LOW);
            break;
        case MOTOR_BACKWARD:
            digitalWrite(MOTOR_RIGHT_FORWARD_PIN,  LOW);
            digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, HIGH);
            break;
    }
}

void driveForward()
{
    bool leftMotorRead = isLeftLineFollowerSensorDetected();
    bool rightMotorRead = isRightLineFollowerSensorDetected();
    

    if (leftMotorRead && rightMotorRead || !leftMotorRead && !rightMotorRead){
        drive(MOTOR_FORWARD, MOTOR_FORWARD);
        return;
    } else if (leftMotorRead) {
        drive(MOTOR_FORWARD, MOTOR_STOP);
        return;
    } else if (rightMotorRead) {
        drive(MOTOR_STOP, MOTOR_FORWARD);
        return;
    }
    // digitalWrite(MOTOR_LEFT_FORWARD_PIN,  !leftMotorRead);
    // digitalWrite(MOTOR_LEFT_BACKWARD_PIN, leftMotorRead);
    // digitalWrite(MOTOR_RIGHT_FORWARD_PIN,  !rightMotorRead);
    // digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, rightMotorRead);
}


void turnLeft(bool isForwarding = true)
{
    bool rightSensorExitedFirstLine = false;
    bool rightSensorEnteredSecondLine = false;
    bool rightSensorExitedSecondLine = false;

    while (!rightSensorExitedSecondLine)
    {
        // Read from the right sensor
        int rightSensorValue = isRightIntersectionSensorDetected();

        // Check if the right sensor has exited the first line
        if (!rightSensorExitedFirstLine && !rightSensorValue) {
            rightSensorExitedFirstLine = true;
        }

        // Check if the right sensor has entered the second line
        if (rightSensorExitedFirstLine && !rightSensorEnteredSecondLine && rightSensorValue) {
            rightSensorEnteredSecondLine = true;
        }

        // Check if the right sensor has exited the second line
        if (rightSensorEnteredSecondLine && !rightSensorExitedSecondLine && !rightSensorValue) {
            rightSensorExitedSecondLine = true;
        }

        // Turn AGV left
        if (isForwarding) {
            drive(MOTOR_FORWARD, MOTOR_BACKWARD);
        } else {
            drive(MOTOR_FORWARD, MOTOR_STOP);
        }
    }
}

void turnRight(bool isForwarding = true)
{
    bool leftSensorExitedFirstLine = false;
    bool leftSensorEnteredSecondLine = false;
    bool leftSensorExitedSecondLine = false;

    while (!leftSensorExitedSecondLine)
    {
        // Read from the left sensor
        int leftSensorValue = isLeftIntersectionSensorDetected();

        // Check if the left sensor has exited the first line
        if (!leftSensorExitedFirstLine && !leftSensorValue) {
            leftSensorExitedFirstLine = true;
        }

        // Check if the left sensor has entered the second line
        if (leftSensorExitedFirstLine && !leftSensorEnteredSecondLine && leftSensorValue) {
            leftSensorEnteredSecondLine = true;
        }

        // Check if the left sensor has exited the second line
        if (leftSensorEnteredSecondLine && !leftSensorExitedSecondLine && !leftSensorValue) {
            leftSensorExitedSecondLine = true;
        }

        // Turn AGV right
        if (isForwarding) {
            drive(MOTOR_BACKWARD, MOTOR_FORWARD);
        } else {
            drive(MOTOR_STOP, MOTOR_BACKWARD);
        }
    }
}

bool isThereIntersection()
{
    diags.intersectionDetected = isLeftIntersectionSensorDetected() && isRightIntersectionSensorDetected();
    return diags.intersectionDetected;
}

void updateCurrentPosition()
{
    if (path == NULL || path->parent == NULL) {
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

    if(last != NULL) {
        delete last;
        secondToLast->parent = NULL;
    } else {
        delete secondToLast;
        thirdToLast->parent = NULL;
    }

    diags.nextDirection = getNextDirection(thirdToLast, secondToLast);

    printPath(path);
    Serial.println("Updated current position: ");
    Serial.print(secondToLast->position[0]);
    Serial.print(", ");
    Serial.println(secondToLast->position[1]);
    Serial.print("Next direction: ");
    printDirection(diags.nextDirection);

}

int getNextDirection(Node* nextPosition, Node* currentPosition){
    int deltaX = nextPosition->position[1] - currentPosition->position[1];
    int deltaY = nextPosition->position[0] - currentPosition->position[0];

    if (deltaX == 2) {
        return EAST_DIRECTION;
    } else if (deltaX == -2) {
        return WEST_DIRECTION;
    } else if (deltaY == 2) {
        return SOUTH_DIRECTION;
    } else if (deltaY == -2) {
        return NORTH_DIRECTION;
    }
}

void turnToNewDirection(bool isForwarding = true)
{
    int directionDifference = diags.nextDirection - diags.currentDirection;

    if (directionDifference == 1 || directionDifference == -3) {
        turnRight(isForwarding);
    } else if (directionDifference == -1 || directionDifference == 3) {
        turnLeft(isForwarding);
    }

    diags.currentDirection = diags.nextDirection;
    Serial.print("Turned to new direction: ");
    printDirection(diags.currentDirection);
}

void printDirection(int direction)
{
    switch (direction)
    {
    case NORTH_DIRECTION:
        Serial.print("NORTH");
        break;
    case EAST_DIRECTION:
        Serial.print("EAST");
        break;
    case SOUTH_DIRECTION:
        Serial.print("SOUTH");
        break;
    case WEST_DIRECTION:
        Serial.print("WEST");
        break;
    }
}

void handleObstacleAndRecalculateRoute()
{
    if (path == NULL || path->parent == NULL) {
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
    if (grid[obstacleX][obstacleY] == '-' || grid[obstacleX][obstacleY] == '|') {
        // Mark the position as an obstacle
        grid[obstacleX][obstacleY] = 'x';

        // Recalculate the route from the current position to the destination
        int start[2] = {last->position[0], last->position[1]};
        int end[2] = {path->position[0], path->position[1]};

        Node *newPath = dijkstra(start, end);

        // Update the path with the new route
        // Make sure to properly delete/free the old path to avoid memory leaks
        deletePath(path);
        path = newPath;
        Serial.println("Recalculated path:");
        printPath(path);

    }
    secondToLast = path;
    last = path->parent;

    {
        secondToLast = last;
        last = last->parent;
    }

    diags.nextDirection = getNextDirection(secondToLast, last);
    Serial.print("Next direction: ");
    printDirection(diags.nextDirection);
}

void deletePath(Node *node)
{
    if (node == NULL) {
        return;
    }

    deletePath(node->parent);
    delete node;
}

void driveBackward()
{
    bool leftMotorRead = isLeftLineFollowerSensorDetected();
    bool rightMotorRead = isRightLineFollowerSensorDetected();
    

    if (leftMotorRead && rightMotorRead || !leftMotorRead && !rightMotorRead){
        drive(MOTOR_BACKWARD, MOTOR_BACKWARD);
        return;
    } else if (leftMotorRead) {
        drive(MOTOR_STOP, MOTOR_BACKWARD);
        return;
    } else if (rightMotorRead) {
        drive(MOTOR_BACKWARD, MOTOR_STOP);
        return;
    }
}

void goBackToIntersection() {
    while (!isThereIntersection()) {
        driveBackward();
    }
}

void loop()
{
    if(diagsThread.shouldRun()){
        diagsThread.run();
    }

    driveForward();
    if (isFrontSensorDetected()) {
        Serial.println("Obstacle detected!");
        goBackToIntersection();
        handleObstacleAndRecalculateRoute();
        if(diags.currentDirection != diags.nextDirection){
            turnToNewDirection(false);
        }
    } else if (isThereIntersection()) {
        updateCurrentPosition();
        if (diags.currentDirection != diags.nextDirection) {
            turnToNewDirection(true);
        } else { 
            while(isThereIntersection()){
                if(diagsThread.shouldRun()){
                    diagsThread.run();
                }
                drive(MOTOR_FORWARD, MOTOR_FORWARD);
            }
        }
    }
}

