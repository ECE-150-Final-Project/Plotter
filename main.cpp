#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <stdexcept>
#include <iostream>

#include <ugpio/ugpio.h>
#include <cmath>
#include <fstream>

/////////////////////////////////////////////////////
// Type Declarations:

struct PolynomialComponent;

struct StatisticalData;

struct Point;

struct ArrayOfPoints;

struct PolynomialFunction;

//For the step motor function. This just makes it so that in the step motor
//function, you can specify if you want to x axis to move, or the y axis to move.
//easy!
enum AXIS {
    X, Y
};
//Also for the step motor function
enum Direction {
    CW, CCW
};

/////////////////////////////////////////////////////
// Function Declarations:

ArrayOfPoints
createArrayOfPolynomialPoints(const PolynomialFunction polynomial, int numPolynomialComponents, const float xMax,
                              const float yMin, const float yMax, const float xMin, const int numPoints);

PolynomialFunction stringToPolynomialFunction(const char input[]);

bool stepMotor(AXIS axis, Direction direction);

bool stepMotor(AXIS axis, Direction direction, const int stepTime);

void requestGPIOAndSetDirectionOutput(int gpio);

void requestGPIOAndSetDirectionInput(int gpio);

//TODO: create LiftPen function and LowerPen functions:
//These need a sleep(10ms) command because they are callback methods that will be run.
bool liftPen();

bool lowerPen();

void startPWM(int gpio, int frequency, int dutyCycle);

void stopPWM(int gpio);

void freeGPIO(int gpio);

bool readGPIO(int gpio);

//Returns the distance it took to go to the point specified.
float gotoPoint(int x, int y);

//Returns the distance it took to go to the point specified.
float gotoPoint(Point point);

bool gotoPointHelper(const int x, const int y);

//TODO: Create drawPolynomial Function:
StatisticalData drawPolynomial(ArrayOfPoints points);

bool gotoZero();

bool logLine(char message[]);

bool openLogFile(const char filename[]);

bool closeLogFile();

/////////////////////////////////////////////////////
// Global Variables:

const int X_AXIS_DIRECTION_GPIO = 0;
const int X_AXIS_STEP_GPIO = 1;
const int Y_AXIS_DIRECTION_GPIO = 5; //SDA I2C GROUP ONION OMEGA
const int Y_AXIS_STEP_GPIO = 4; //SCL I2C GROUP ONION OMEGA

const int SERVO_PIN = 18; //PWM0 GROUP ON ONION OMEGA
const int SERVO_FREQUENCY = 50; //50Hz
const int SERVO_UP_DUTY_CYCLE = 50; //50% duty cycle for up
const int SERVO_DOWN_DUTY_CYCLE = 0; //0% duty cycle for down
const int SERVO_CHANGE_TIME = 100 * 1000; //Time required for the servo to change its position

const int X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO = 45; //TX1 UART1 GROUP ONION OMEGA
const int X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO = 46; //RX1 UART1 GROUP ONION OMEGA
const int Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO = 2;
const int Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO = 3;


const int STEP_TIME = 2 * 1000; //time it takes to step a stepper motor in microseconds.
const float X_MAX = 1650; //The x-limit of the plotter.
const float Y_MAX = 2100; //The y-limit of the plotter.

int currentX = 0; // Assuming the plotter starts at x-origin
int currentY = 0; // Assuming the plotter starts at y-origin

std::ofstream logFile;
const char LOG_FILE_NAME[] = "log_file.txt";

const float SLOPE_PRECISION = 1;

/////////////////////////////////////////////////////
// Function Definitions:

//This is a "polynomial component", if the polynomial function is 2x^2 + 3x + 4, then it contains three "components:"
//2x^2, 3x, and 4. For the 2x^2 thing, the constant is 2, and the exponent is 2. For the 3x, the constant is 3, and the
//exponent is 1. You can guess what 4 is.
struct PolynomialComponent {
    int constant;
    int exponent;
};

//This is going to be used in an array of points that will be used to store all of the points on the polynomial.
struct Point {
    float x;
    float y;
};

struct ArrayOfPoints {
    Point *points;
    int numPoints;
};

struct StatisticalData {
    float lengthOfTime;
    float lengthOfFunction;
};

struct PolynomialFunction {
    PolynomialComponent *components;
    int numComponents;
};

ArrayOfPoints
createArrayOfPolynomialPoints(const PolynomialFunction polynomial, int numPolynomialComponents, const float xMax,
                              const float yMin, const float yMax, const float xMin, const int numPoints) {

    //Close inputs:
    if (xMax <= xMin || yMax <= yMin) {
        ArrayOfPoints failure;
        failure.points = nullptr;
        failure.numPoints = 0;
        return failure;
    }

    ArrayOfPoints points;
    points.numPoints = numPoints;
    points.points = new Point[numPoints];

    //For each point, lets determine an x-value based on windowSize.
    //So, there are numPoints number of points, and we have to have them all equally distributed between xMin and xMax.
    //What is the difference between xMax and xMin?
    float xWindowSize = xMax - xMin;

    //Now find the distance between each of the domain points.
    float deltaX = xWindowSize / numPoints;

    //Now let's set a currentDomainX, and we'll add deltaX to it and set all the points x-values to it.
    //It equals to xMin by default because that's the minimum value we want.
    float currentDomainX = xMin;

    for (int i = 0; i < numPoints; i++) {
        points.points[i].x = currentDomainX;
        //Setting all the y-values to 0 is very important. It allows us to add to y later.
        points.points[i].y = 0;
        currentDomainX += deltaX;
    }

    //Now, for every polynomial component, we shall ADD (not set!) a y-value to every x-value.
    for (int currentPolynomialComponent = 0;
         currentPolynomialComponent < numPolynomialComponents; currentPolynomialComponent++) {
        //For each x-value in the array:
        for (int i = 0; i < numPoints; i++) {
            //We make a yValue that is the pow(points->points[i].x, polynomial[currentPolynomialComponent].exponent)
            float yValue = (float) pow(points.points[i].x,
                                       (double) polynomial.components[currentPolynomialComponent].exponent);
            //Now we scale yValue using the constant.
            yValue *= (float) polynomial.components[currentPolynomialComponent].constant;

            //Then we add our current y value to the point's y value, so that we can add all the polynomial function
            //together.
            points.points[i].y += yValue;
        }
    }

    //Now that we have all the points, we must ensure that they are all valid.
    for (int i = 0; i < numPoints; i++) {
        //Let's see if the y-value fits inside of our window.
        if (points.points[i].y > yMax || points.points[i].y < yMin) {
            points.points[i].y = NAN;
        }
    }

    std::cout << "Before translation:" << std::endl << std::endl;


    for (int i = 0; i < numPoints; i++) {
        std::cout << "Point " << i + 1 << ": (" << points.points[i].x << ", " << points.points[i].y << ")" << std::endl;
    }

    //TODO: translate all the points to have xmin and ymin at (0,0) and scale them to fit inside of the plotter's range of motion.

    //Now, let's translate all these points so that the first x-value is 0, and the lowest y-value is 0.
    //To get where the lowest x-value is, just look at the first item of the array, and then take 0-it and
    //see by how much we need to add to every point to translate the points.
    float translateX = -points.points[0].x;

    //Now, lets find the minimum value of y:
    float translateY = -yMin;

    //Now Lets actually translate them:
    for (int i = 0; i < numPoints; i++) {
        points.points[i].x += translateX;
        if (!std::isnan(points.points[i].y)) {
            points.points[i].y += translateY;
        }
    }

    std::cout << "After translation:" << std::endl << std::endl;
    for (int i = 0; i < numPoints; i++) {
        std::cout << "Point " << i + 1 << ": (" << points.points[i].x << ", " << points.points[i].y << ")" << std::endl;
    }

    //Now let's scale them!
    //All x-values must be scaled from the range of xMin to xMax to the range of 0-X_MAX
    for (int i = 0; i < numPoints; i++) {
        //Let's first do that by normalizing the data, which is getting it from its range to 0-1.
        //Then afterward, we'll multiply that by X_MAX.
        points.points[i].x = (points.points[i].x - 0) / (xMax - xMin);
        points.points[i].x *= X_MAX;
        if (!std::isnan(points.points[i].y)) {
            points.points[i].y = (points.points[i].y - 0) / (yMax - yMin);
            points.points[i].y *= Y_MAX;
        }
    }

    std::cout << "After normalization and then scaling:" << std::endl << std::endl;
    for (int i = 0; i < numPoints; i++) {
        std::cout << "Point " << i + 1 << ": (" << points.points[i].x << ", " << points.points[i].y << ")" << std::endl;
    }

    std::cout << "End of whatsit function:" << std::endl << std::endl;
    return points;
}

PolynomialFunction stringToPolynomialFunction(const char input[]) {
    PolynomialFunction function;
    function.components = new PolynomialComponent[50];
    function.numComponents = 0;
    PolynomialFunction rejected;
    rejected.components = NULL;
    rejected.numComponents = 0;

    int SizeofString = 1;
    int m = 0;
    while (input[m] != 0) {
        SizeofString++;
        m++;
    }

    bool Done = false;
    int iterator = 0;
    int i = 0;
    int size = 0;
    int calculatedvalue = 0;
    bool notNumber = false;
    int k = (size - 1);
    int sum = 0;
    char temp[SizeofString];
    while (!Done) {

        bool negativeSign = false;
        PolynomialComponent Section;

        if ((input[iterator] == '+') || input[iterator] == '-') {
            if (input[iterator] == '-') {
                negativeSign = true;
            }
            iterator++;
        }
        if (input[iterator] == 'x' || input[iterator] == 'X') {
            if (negativeSign) {
                Section.constant = -1;
            } else {
                Section.constant = 1;
            }
        }
        if (input[iterator] == '0') {
            while ((input[iterator] != '+') || (input[iterator] != '-')) {
                iterator++;
            }
        } else {
////////////////////////////////////////////////////////////////////////////////////////
            if ((input[iterator] >= '1') && (input[iterator] <= '9')) {
                /// Initialize to all nulls for the meme ///
                for (int i = 0; i < SizeofString; i++) {
                    temp[i] = 0;
                }

                /// Inputing the values into a temp array so we may turn them into ints///
                notNumber = false;
                for (int i = 0; !notNumber; i++) {
                    temp[i] = input[iterator];
                    iterator++;

                    /// If the next value of of iterator is an X, then the for loop will end ///
                    /// If the value after the numbers are not X's then we know that it is an incorrect input to what we have ///
                    if ((input[iterator] == 'x') || (input[iterator] == 'X')) {
                        notNumber = true;
                    }
                    if (input[iterator] == 0) {
                        notNumber = true;
                    }
                }
                /// finding the size fo the temp array ///
                size = 0;
                while (temp[size] != 0) {
                    size++;
                }
                /// Time to convert to int///
                k = (size - 1);
                calculatedvalue = 0;
                sum = 0;
                for (int i = 0; i < size; i++) {
                    calculatedvalue = ((int) temp[i] - '0') * pow(10, k);
                    sum = calculatedvalue + sum;
                    k = (k - 1);
                    /// Overflow then function fails///
                    if (sum < 0) {
                        delete function.components;
                        return rejected;
                    }
                }
                ///Negative is true sets the sum negative ///
                if (negativeSign) {
                    sum = -sum;
                }
                Section.constant = sum;
            }
        }
/////////////////////////////////////////////////////////////////
        if (input[iterator + 1] == '^') {
            iterator += 2;
            for (int i = 0; i < SizeofString; i++) {
                temp[i] = 0;
            }

            notNumber = false;
            for (int i = 0; !notNumber; i++) {
                temp[i] = input[iterator];
                /// If the next value of of iterator is an + or -, then the for loop will end ///
                /// If the value after the numbers are not +'s or  then we know that it is an incorrect input to what we have ///
                if ((input[iterator + 1] == '+') || (input[iterator + 1] == '-') || (input[iterator + 1] == 0)) {
                    notNumber = true;
                }
                iterator++;
            }

            if (input[iterator] == 0) {
                Done = true;
                /// finding the size fo the temp array ///
                int size = 0;
                while (temp[size] != 0) {
                    size++;
                }
                /// Time to convert to int///
                calculatedvalue = 0;
                k = (size - 1);
                sum = 0;
                for (int i = 0; i < size; i++) {
                    calculatedvalue = ((int) temp[i] - '0') * pow(10, k);
                    sum = calculatedvalue + sum;
                    k = (k - 1);
                    /// Overflow then function fails///
                    if (sum < 0) {
                        delete function.components;
                        return rejected;
                    }
                }
                Section.exponent = sum;
            } else {
                /// finding the size fo the temp array ///
                int size = 0;
                while (temp[size] != 0) {
                    size++;
                }
                /// Time to convert to int///
                calculatedvalue = 0;
                k = (size - 1);
                sum = 0;
                for (int i = 0; i < size; i++) {
                    calculatedvalue = ((int) temp[i] - '0') * pow(10, k);
                    sum = calculatedvalue + sum;
                    k = (k - 1);
                    /// Overflow then function fails///
                    if (sum < 0) {
                        delete function.components;
                        return rejected;
                    }
                }
                Section.exponent = sum;
            }
            ///
        } else if ((input[iterator] == 'x') || (input[iterator] == 'X')) {
            Section.exponent = 1;
            iterator++;
        } else {
            Section.exponent = 0;
            if (input[iterator] == 0) {
                Done = true;
            } else {
                iterator++;
            }
        }
        function.components[i] = Section;
        function.numComponents++;
        i++;
        if (input[iterator] == 0) {
            Done = true;
        }
    }
    return function;
}

bool stepMotor(AXIS axis, Direction direction) {

    //Local step and direction GPIOs so that they can be set based on inputs.
    int stepGPIO;
    int dirGPIO;
    switch (axis) {
        case X:
            stepGPIO = X_AXIS_STEP_GPIO;
            dirGPIO = X_AXIS_DIRECTION_GPIO;
            break;
        case Y:
            stepGPIO = Y_AXIS_STEP_GPIO;
            dirGPIO = Y_AXIS_DIRECTION_GPIO;
            break;
    }

    switch (direction) {
        case CW:

            //Check to see if the limit switch is yelling at you:
            //Gotta determine if it's the X or the Y axis first though.
            //IF Y and if CW, then use Limit Y MAX.
            if (stepGPIO == Y_AXIS_STEP_GPIO && readGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, Read Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO Limit switch true."
                          << std::endl;
                if (readGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }
            //IF X and CW, then use Limit X MAX.
            if (stepGPIO == X_AXIS_STEP_GPIO && readGPIO(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO Read Limit switch true."
                          << std::endl;
                if (readGPIO(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }

            //set directionGPIO to HIGH, because GND is CW.
            gpio_set_value(dirGPIO, 1);
            break;
        case CCW:

            //Check to see if the limit switch is yelling at you:
            //Gotta determine if it's the X or the Y axis first though.
            //IF Y and if CCW, then use Limit Y MIN.
            if (stepGPIO == Y_AXIS_STEP_GPIO && readGPIO(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, Read Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO Limit switch true."
                          << std::endl;
                if (readGPIO(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }
            //IF X and CCW, then use Limit X MIN.
            if (stepGPIO == X_AXIS_STEP_GPIO && readGPIO(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, Read X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO Limit switch true."
                          << std::endl;
                if (readGPIO(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }

            //set directionGPIO to GND, because HIGH is CCW.
            gpio_set_value(dirGPIO, 0);
            break;
    }

    //sleep for a few more milliseconds, because this function will probably
    //be called consecutively all the time with no breaks.
    //We also need to allow time for the direction pin to charge up.
    usleep(STEP_TIME);

    //pulse step on:
    gpio_set_value(stepGPIO, 1);
    //sleep for a few milliseconds, because you need to let the coils charge etc.
    usleep(STEP_TIME);
    //pull step to GND, because its GND activated:
    gpio_set_value(stepGPIO, 0);
    return true;
}

bool stepMotor(AXIS axis, Direction direction, const int stepTime) {

    //Local step and direction GPIOs so that they can be set based on inputs.
    int stepGPIO;
    int dirGPIO;
    switch (axis) {
        case X:
            stepGPIO = X_AXIS_STEP_GPIO;
            dirGPIO = X_AXIS_DIRECTION_GPIO;
            break;
        case Y:
            stepGPIO = Y_AXIS_STEP_GPIO;
            dirGPIO = Y_AXIS_DIRECTION_GPIO;
            break;
    }

    switch (direction) {
        case CW:

            //Check to see if the limit switch is yelling at you:
            //Gotta determine if it's the X or the Y axis first though.
            //IF Y and if CW, then use Limit Y MAX.
            if (stepGPIO == Y_AXIS_STEP_GPIO && readGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, Read Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO Limit switch true."
                          << std::endl;
                if (readGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }
            //IF X and CW, then use Limit X MAX.
            if (stepGPIO == X_AXIS_STEP_GPIO && readGPIO(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO Read Limit switch true."
                          << std::endl;
                if (readGPIO(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }

            //set directionGPIO to HIGH, because GND is CW.
            gpio_set_value(dirGPIO, 1);
            break;
        case CCW:

            //Check to see if the limit switch is yelling at you:
            //Gotta determine if it's the X or the Y axis first though.
            //IF Y and if CCW, then use Limit Y MIN.
            if (stepGPIO == Y_AXIS_STEP_GPIO && readGPIO(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, Read Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO Limit switch true."
                          << std::endl;
                if (readGPIO(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }
            //IF X and CCW, then use Limit X MIN.
            if (stepGPIO == X_AXIS_STEP_GPIO && readGPIO(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO)) {
                std::cout << "Failed to step motor, Read X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO Limit switch true."
                          << std::endl;
                if (readGPIO(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO) == -1) {
                    perror("X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
                    throw std::exception();
                }
                return false;
            }

            //set directionGPIO to GND, because HIGH is CCW.
            gpio_set_value(dirGPIO, 0);
            break;
    }

    //sleep for a few more milliseconds, because this function will probably
    //be called consecutively all the time with no breaks.
    //We also need to allow time for the direction pin to charge up.
    usleep(stepTime);

    //pulse step on:
    gpio_set_value(stepGPIO, 1);
    //sleep for a few milliseconds, because you need to let the coils charge etc.
    usleep(stepTime);
    //pull step to GND, because its GND activated:
    gpio_set_value(stepGPIO, 0);
    return true;
}

StatisticalData drawPolynomial(ArrayOfPoints points) {

    StatisticalData statisticalData;
    statisticalData.lengthOfFunction = 0;

    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);

    //Get the time relative to the hour of the day. That should be good enough.
    statisticalData.lengthOfTime = (now->tm_hour * 60 * 60) + (now->tm_min * 60) + (now->tm_sec);

    //First of all, lift the pen, and go to zero!
    liftPen();
    gotoZero();
//Print everything out human readable:
    for (int i = 0; i < points.numPoints; i++) {
        std::cout << "Point " << i + 1 << ": (" << points.points[i].x << ", " << points.points[i].y << ")" << std::endl;
    }

    gotoZero();
    std::cout << std::endl;
    bool liftedPen = true;
    bool gotToValidPoint = false;
    for (int i = 0; i < points.numPoints; i++) {
        std::cout << "Going to point: (" << (int) points.points[i].x << ", " << (int) points.points[i].y << ")"
                  << std::endl;
        if (!std::isnan(points.points[i].y)) {
            if (liftedPen && gotToValidPoint) {
                lowerPen();
                liftedPen = false;
            }
            statisticalData.lengthOfFunction += gotoPoint(points.points[i]);
            gotToValidPoint = true;
        } else {
            if (liftedPen) {
                liftPen();
            }
            liftedPen = true;
        }
    }

    t = time(0);   // get time now
    now = localtime(&t);

    statisticalData.lengthOfTime = (((now->tm_hour * 60 * 60) + (now->tm_min * 60) + (now->tm_sec)) -
                                    statisticalData.lengthOfTime);

    return statisticalData;
}

void requestGPIOAndSetDirectionOutput(int gpio) {
    int gpioRequest;
    int gpioDirection;

    // check if gpio is already requested
    if ((gpioRequest = gpio_is_requested(gpio)) < 0) {
        perror("gpio_is_requested");
        throw std::exception();
    }

    // request the gpio
    if (!gpioRequest) {
        printf("> exporting gpio\n");
        if ((gpioDirection = gpio_request(gpio, NULL)) < 0) {
            perror("gpio_request");
            throw std::exception();
        }
    }

//     set to input direction
//    printf("> setting to input\n");
//    if ((gpioDirection = gpio_direction_input(gpio)) < 0) {
//        perror("gpio_direction_input");
//    }

    // set to output direction:
    printf("> setting to output\n");
    if ((gpioDirection = gpio_direction_output(gpio, 0)) < 0) {
        perror("gpio_direction_output");
    }
}

void requestGPIOAndSetDirectionInput(int gpio) {
    int gpioRequest;
    int gpioDirection;

    // check if gpio is already requested
    if ((gpioRequest = gpio_is_requested(gpio)) < 0) {
        perror("gpio_is_requested");
        throw std::exception();
    }

    // request the gpio
    if (!gpioRequest) {
        printf("> exporting gpio\n");
        if ((gpioDirection = gpio_request(gpio, NULL)) < 0) {
            perror("gpio_request");
            throw std::exception();
        }
    }

//     set to input direction
//    printf("> setting to input\n");
//    if ((gpioDirection = gpio_direction_input(gpio)) < 0) {
//        perror("gpio_direction_input");
//    }

    // set to output direction:
    printf("> setting to input\n");
    if ((gpioDirection = gpio_direction_input(gpio)) < 0) {
        perror("gpio_direction_input");
    }
}

void freeGPIO(int gpio) {
    if (gpio_free(gpio) < 0) {
        perror("freeGPIO had an error.\n");
    }
}

//DONE: Create overload for this function that accepts a struct Point.
float gotoPoint(int x, int y) {
    int oldX = currentX; // OldX is the original y-coordinate of the motor
    int oldY = currentY; // OldY is the original y-coordinate of the motor
//    float SLOPE_PRECISION = 1; // This is the maximum amount of precision I think we should allow
    float masterslope = ((float) y - (float) oldY) / ((float) x -
                                                      (float) oldX); // This slope is the slope that we're always checking with.  Eqn of slope is (y2-y1)/(x2-x1
    float currentslope = ((float) y - (float) currentY) /
                         ((float) x - (float) currentX); // This is the slope that is recalculated with every new step.
    Direction directionX; // Variable helps specify the direction that the motor will always be going, there could be a better way, but for now I have specified an individual direction for both the x and y
    Direction directionY;
    AXIS axis;
    int changeofX;
    int changeofY;
//    double SLOPE_PRECISION =( ( ((float)y - (float)oldY)/((float)x - (float)oldX-1) )-( ((float)y - (float)oldY)/((float)x - (float)oldX) ) * 100);
//    double SLOPE_PRECISION = 1;

    if (x > currentX) { // This determines the original X-direction of the motor.
        directionX = CW;
        changeofX = 0;
    } else if (x < currentX) {
        directionX = CCW;
        changeofX = 1;
    }

    if (y > currentY) { // This determines the original Y-direction of the motor.
        directionY = CW;
        changeofY = 0;
    } else if (y < currentY) {
        directionY = CCW;
        changeofY = 1;
    }

    if (currentX == x && currentY == y) { // For the scenario of the (x,y) being a single point
        return true;
    }

    if (currentX == x) { // For the scenario of the (x,y) resulting in a vertical line
        while (currentY != y) {
            axis = Y;
            stepMotor(axis, directionY);
            currentY = currentY + (-2 * changeofY + 1);
        }
        return true;
    }

    if (currentY == y) { // For the scenario of the (x,y) resulting in a horizontal line
        while (currentX != x) {
            axis = X;
            stepMotor(axis, directionX);
            currentX = currentX + (-2 * changeofX + 1);
        }
        return true;
    }

    while (x != currentX || y != currentY) {
//        std::cout << "x:" << x << " currentX:" << currentX << std::endl;
//        std::cout << "y:" << y << " currentY:" << currentY << std::endl;

        while (currentslope >= (masterslope - SLOPE_PRECISION) && currentslope <= (masterslope + SLOPE_PRECISION) &&
               x !=
               currentX) { // I think this is right.  Exits loop when the currentslope decreases past a critical point.  Should specifiy this while loop is for the X increases
            axis = X;
            stepMotor(axis, directionX); // This should make one xs - step towards the desired point.
            currentX = currentX + (-2 * changeofX +
                                   1); // The new currentX location. The "-2*directionX + 1" is the way I can determine whether it increases or decreases. lol its jokes
            currentslope = ((float) y - (float) currentY) / ((float) x - (float) currentX);
//            std::cout << "suck this:" << currentslope << "x" << masterslope << std::endl;
        }
        while (currentslope < (masterslope - SLOPE_PRECISION) || currentslope > (masterslope +
                                                                                 SLOPE_PRECISION)) { // I think this is right.  Exits loop when the currentslope decreases past a critical point.  Should specifiy this while loop is for the Y increases
            axis = Y;
            stepMotor(axis, directionY); // This should make one Y-step towards the desired point.
            currentY = currentY + (-2 * changeofY +
                                   1); // The new currentY location. The "-2*directionY + 1" is the way I can determine whether it increases or decreases. lol its jokes
            currentslope = ((float) y - (float) currentY) / ((float) x - (float) currentX);
//            std::cout << "suck this:" << currentslope << "y" << masterslope - SLOPE_PRECISION << std::endl;
        }

        /* if (x == currentX){
             axis = Y;
             stepMotor(axis, directionY); // This should make one Y-step towards the desired point.
             currentY = currentY + (-2 * changeofY + 1);
         }*/

    }


    return true;
}

float gotoPoint(Point point) {
    int oldX = currentX; // OldX is the original y-coordinate of the motor
    int oldY = currentY; // OldY is the original y-coordinate of the motor

    float dx = point.x - oldX;
    float dy = point.y - oldY;

    int x = currentX;

    for (; x < point.x; x++) {
        float y = (float) oldY + (float) dy * ((float) x - (float) oldX) / (float) dx;
        gotoPointHelper(x, (int) y);
    }

    //Calculate distance using pythagorean theorem:
    return (float) sqrt(pow(dx, 2) + pow(dy, 2));
}

//This is a dumb goto point function that simply goes in the shape of an L to get to a point.
//It's only going to be used for very small x or y increments.
//Tested successfully.
bool gotoPointHelper(const int x, const int y) {
    Direction directionX; // Variable helps specify the direction that the motor will always be going, there could be a better way, but for now I have specified an individual direction for both the x and y
    Direction directionY;
    AXIS axis;
    int changeofX;
    int changeofY;
//    double SLOPE_PRECISION =( ( ((float)y - (float)oldY)/((float)x - (float)oldX-1) )-( ((float)y - (float)oldY)/((float)x - (float)oldX) ) * 100);
//    double SLOPE_PRECISION = 1;

    if (x > currentX) { // This determines the original X-direction of the motor.
        directionX = CW;
        changeofX = 0;
    } else if (x < currentX) {
        directionX = CCW;
        changeofX = 1;
    }

    if (y > currentY) { // This determines the original Y-direction of the motor.
        directionY = CW;
        changeofY = 0;
    } else if (y < currentY) {
        directionY = CCW;
        changeofY = 1;
    }

    if (currentX == x && currentY == y) { // For the scenario of the (x,y) being a single point
        return true;
    }

    if (currentX == x) { // For the scenario of the (x,y) resulting in a vertical line
        while (currentY != y) {
            axis = Y;
            stepMotor(axis, directionY);
            currentY = currentY + (-2 * changeofY + 1);
        }
        return true;
    }

    if (currentY == y) { // For the scenario of the (x,y) resulting in a horizontal line
        while (currentX != x) {
            axis = X;
            stepMotor(axis, directionX);
            currentX = currentX + (-2 * changeofX + 1);
        }
        return true;
    }

    while (currentX != x) {
        axis = X;
        stepMotor(axis, directionX);
        currentX = currentX + (-2 * changeofX + 1);
    }
    while (currentY != y) {
        axis = Y;
        stepMotor(axis, directionY);
        currentY = currentY + (-2 * changeofY + 1);
    }
    return true;
}

//Tested successfully.
bool gotoZero() {
    while (stepMotor(X, CCW, 1 * 1000) || stepMotor(Y, CCW, 1 * 1000));
    return true;
}

bool readGPIO(int gpio) {
    //First check the direction of the GPIO:
//    int direction = gpio_get_direction(gpio);
//    if (direction == GPIOF_DIR_OUT) {
//        perror("Can't read GPIO, gpio is set to DIR_OUT");
//        throw std::exception();
//    } else if (direction == GPIOF_DIR_IN) {
//    std::cout << "Read GPIO: " << gpio << ", Value: " << gpio_get_value(gpio) << std::endl;
    return (bool) gpio_get_value(gpio);
//    } else {
//        perror("Can't read GPIO, You don't understand GPIOF_DIR_OUT vs GPIOF_DIR_IN");
//        throw std::exception();
//    }
}

void startPWM(int gpio, int frequency, int dutyCycle) {
    int gpioRequest;
    if ((gpioRequest = gpio_is_requested(gpio)) < 0) {
        perror("gpio_is_requested");
        throw std::exception();
    }

    std::string command = "fast-gpio pwm ";
    std::string totalCommand =
            command + std::to_string(gpio) + " " + std::to_string(frequency) + " " + std::to_string(dutyCycle);
    system(totalCommand.c_str());
}

void stopPWM(int gpio) {
    std::string command = "fast-gpio set ";
    std::string totalCommand = command + std::to_string(gpio) + " 0";
    system(totalCommand.c_str());
}

bool liftPen() {
    std::cout << "Lifted Pen." << std::endl;
    startPWM(SERVO_PIN, SERVO_FREQUENCY, SERVO_UP_DUTY_CYCLE);
    usleep(SERVO_CHANGE_TIME);
    return true;
}

bool lowerPen() {
    std::cout << "Lowered Pen." << std::endl;
    startPWM(SERVO_PIN, SERVO_FREQUENCY, SERVO_DOWN_DUTY_CYCLE);
    usleep(SERVO_CHANGE_TIME);
    return true;
}

bool logLine(char message[]) {
    logFile << "Writing this to a file.\n";
}

bool openLogFile(const char filename[]) {
    logFile.open(filename);
}

bool closeLogFile() {
    logFile.close();
}

int main(const int argc, const char *const argv[]) {

    std::cout << "Did you remember to set uart1 to gpio?" << std::endl;

    requestGPIOAndSetDirectionOutput(X_AXIS_DIRECTION_GPIO);
    requestGPIOAndSetDirectionOutput(X_AXIS_STEP_GPIO);
    requestGPIOAndSetDirectionOutput(Y_AXIS_DIRECTION_GPIO);
    requestGPIOAndSetDirectionOutput(Y_AXIS_STEP_GPIO);

    requestGPIOAndSetDirectionInput(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO);
    requestGPIOAndSetDirectionInput(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO);
    requestGPIOAndSetDirectionInput(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO);
    requestGPIOAndSetDirectionInput(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO);

    openLogFile(LOG_FILE_NAME);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////createArrayOfPolynomialPoints testing:
//    int numPolynomialComponents = 2;
//    PolynomialComponent function[numPolynomialComponents];
//
//    //Manually enter them because Erik's parse doesn't work quite yet.
//    function[0].constant = 3;
//    function[0].exponent = 2;
//    //Second item:
//    function[1].constant = 2;
//    function[1].exponent = 1;
//
//    float xMin = -2;
//    float xMax = 2;
//    float yMin = -1;
//    float yMax = 4;
//
//    int numPoints = 10;
//
//    ArrayOfPoints arrayOfPoints = createArrayOfPolynomialPoints(function, numPolynomialComponents, xMax, yMin, yMax,
//                                                                xMin, numPoints);
////Print everything out human readable:
//    for(int i = 0; i < numPoints; i++) {
//        std::cout << "Point " << i+1 << ": (" << arrayOfPoints.points[i].x << ", " << arrayOfPoints.points[i].y << ")" << std::endl;
//    }
//
////Print everything out machine readable:
//    for(int i = 0; i < numPoints; i++) {
//        std::cout << arrayOfPoints.points[i].x << ", " << arrayOfPoints.points[i].y << std::endl;
//    }



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////drawPolynomial testing:

    if (argc < 5) {
        std::cout << "Usage: <\"ax^b+cx^d+...\">, <xMin>, <xMax>, <yMin>, <yMax>," << std::endl;
        return 0;
    }


    PolynomialFunction function = stringToPolynomialFunction(argv[1]);

    for (int i = 0; i < function.numComponents; i++) {
        std::cout << "Polynomial Component " << function.components[i].constant << std::endl;
        std::cout << "Polynomial Exponoent " << function.components[i].exponent << std::endl;
    }

//    float xMin = -2;
//    float xMax = 2;
//    float yMin = -1;
//    float yMax = 4;

    float xMin = atoi(argv[2]);
    float xMax = atoi(argv[3]);
    float yMin = atoi(argv[4]);
    float yMax = atoi(argv[5]);

    int numPoints = 100;

    ArrayOfPoints arrayOfPoints = createArrayOfPolynomialPoints(function, function.numComponents, xMax, yMin, yMax,
                                                                xMin, numPoints);
//Print everything out human readable:
    for (int i = 0; i < numPoints; i++) {
        std::cout << "Point " << i + 1 << ": (" << arrayOfPoints.points[i].x << ", " << arrayOfPoints.points[i].y << ")"
                  << std::endl;
    }

//Print everything out machine readable:
    for (int i = 0; i < numPoints; i++) {
        std::cout << arrayOfPoints.points[i].x << ", " << arrayOfPoints.points[i].y << std::endl;
    }

    StatisticalData statisticalData = drawPolynomial(arrayOfPoints);

    std::cout << "\nStatisticalData: " << std::endl;
    std::cout << "Length of function: " << (statisticalData.lengthOfFunction * 0.2278) / 10.0 << "cm" << std::endl;
    std::cout << "Length of time to draw function: " << statisticalData.lengthOfTime << "s" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////logFile Testing:
    char logMessage[] = "LogTheThing.";
    logLine(logMessage);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////Servo (Pen) Motor testing:
//    liftPen();
//    lowerPen();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////StepMotor testing:
//    Direction direction;
//    AXIS axis;
//
//    int numSteps = 0;
//
//    if (!(strcmp(argv[1], "X"))) {
//        axis = X;
//    } else if (!(strcmp(argv[1], "Y"))) {
//        axis = Y;
//    } else {
//        axis = Y;
//    }
//
//    if (!(strcmp(argv[2], "CCW"))) {
//        direction = CCW;
//    } else if (!(strcmp(argv[2], "CW"))) {
//        direction = CW;
//    } else {
//        direction = CCW;
//    }
//
//
//    // Set the gpio from positive to negative 20 times
//    printf("> begin stepping the motor!\n");
//    for (int i = 0; i < 10000; i++) {
//        if (stepMotor(axis, direction)) {
//            numSteps++;
//        } else {
//            std::cout << "numSteps: " << numSteps << std::endl;
//        }
//
//    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////Determine total size of grid testing:
//    gotoZero();
//    Direction direction;
//    AXIS axis;
//
//    int numSteps = 0;
//
//    if (!(strcmp(argv[1], "X"))) {
//        axis = X;
//    } else if (!(strcmp(argv[1], "Y"))) {
//        axis = Y;
//    } else {
//        axis = Y;
//    }
//
//    if (!(strcmp(argv[2], "CCW"))) {
//        direction = CCW;
//    } else if (!(strcmp(argv[2], "CW"))) {
//        direction = CW;
//    } else {
//        direction = CCW;
//    }
//
//
//    // Set the gpio from positive to negative 20 times
//    printf("> begin stepping the motor!\n");
//    for (int i = 0; i < 10000; i++) {
//        if (stepMotor(axis, direction)) {
//            numSteps++;
//        } else {
//            std::cout << "numSteps: " << numSteps << std::endl;
//            return 0;
//        }
//
//    }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////GotoZero testing:
//    gotoZero();


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////Goto testing:
//    int x = atoi(argv[1]);
//    int y = atoi(argv[2]);
//    gotoPoint(x, y);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////More goto testing:
//    gotoPoint(x - 50, y - 40);
//    gotoPoint(x + 60, y - 40);
//    gotoZero();

    closeLogFile();

    std::cout << "Free GPIOs that are Outputs: " << std::endl;
    freeGPIO(X_AXIS_DIRECTION_GPIO);
    freeGPIO(X_AXIS_STEP_GPIO);
    freeGPIO(Y_AXIS_DIRECTION_GPIO);
    freeGPIO(Y_AXIS_STEP_GPIO);

    std::cout << "Free GPIOs that are Inputs: " << std::endl;
    freeGPIO(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO);
    freeGPIO(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO);
    freeGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO);
    freeGPIO(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO);


    return 0;
}

//DONE: Limit switches
//DONE: configure input gpios, for limit switches
//TODO: figure out how you're gonna interface with servo motors.
//DONE: motors go backwards to how they're supposed to.
//Make array of points that represent the polynomial.
