#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>
#include <iostream>

#include <ugpio/ugpio.h>

/////////////////////////////////////////////////////
// Type Declarations:

struct PolynomialComponent;

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


PolynomialComponent *stringToPolynomialFunction(const char input[]);

/**
 * Steps the motor a single 1.8 degree step
 * @param direction A boolean value that determines if you want the motor to go clockwise or not.
 * @param axis The gpio pin your motor direction is attached to.
 * @param gpioStp The gpio pin your motor stp is attached to.
 */
bool stepMotor(AXIS axis, Direction direction);

void requestGPIOAndSetDirectionOutput(int gpio);

void freeGPIO(int gpio);

bool readGPIO(int gpio);

bool gotoXYpoint(int x, int y);

/////////////////////////////////////////////////////
// Global Variables:

const int X_AXIS_DIRECTION_GPIO = 0;
const int X_AXIS_STEP_GPIO = 1;
const int Y_AXIS_DIRECTION_GPIO = 2;
const int Y_AXIS_STEP_GPIO = 3;

const int X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO = 6;
const int X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO = 19;
const int Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO = 18;
const int Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO = 11;


const int STEP_TIME = 10000; //time it takes to step a stepper motor in microseconds.

int currentX = 0; // Assuming the plotter starts at x-origin
int currentY = 0; // Assuming the plotter starts at y-origin

/////////////////////////////////////////////////////
// Function Definitions:

//This is a "polynomial component", if the polynomial function is 2x^2 + 3x + 4, then it contains three "components:"
//2x^2, 3x, and 4. For the 2x^2 thing, the constant is 2, and the exponent is 2. For the 3x, the constant is 3, and the
//exponent is 1. You can guess what 4 is.
struct PolynomialComponent {
    int constant;
    int exponent;
};

//TODO: Erik needs to make a state machine that converts the input into an array of constants and exponents.
//Here is the interface that you will implement:
//This function converts a polynomial in string form to an array of PolynomialComponents. Its return type is PolynomialComponent*, which is an array of polynomial Components.
PolynomialComponent *stringToPolynomialFunction(const char input[]) {

    //Loop through the string and determine its size. This will be used for the dynamic memory allocation later.
    int sizeOfString = 0;

//    struct PolynomialComponent* polynomialFunction[sizeOfString];
    //Note that this thing ^ is created on the stack. If we return it at the end of the function, all the memory inside
    //it disappears. This is bad. To fix this, what we want to do is do dynamic memory allocation, we want polynomialFunction = new PolynomialFunction[sizeOfString];
    //But, we can't do that because we're programming in the wonderful language of C. We need to use malloc and free. Malloc is like = new
    //and free is like delete in c++. Go online and learn:
    //malloc, and
    //free.

    //here is an example of how we're gonna do memory allocation. Make sure I did this right!!!
    //Note that this over-allocated memory, because it allocates memory for characters like 'x', '^', and '+', etc, that are in the string.
//    struct PolynomialComponent *polynomialFunction = malloc(sizeOfString * sizeof(struct PolynomialComponent));
    PolynomialComponent *polynomialFunction = new PolynomialComponent[sizeOfString];

    //... convert string to polynomialComponent array ...

    return polynomialFunction;
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

bool gotoXYpoint(const int x, const int y) {
    int oldX = currentX; // OldX is the original y-coordinate of the motor
    int oldY = currentY; // OldY is the original y-coordinate of the motor
    float precision = 0.1; // This is the maximum amount of precision I think we should allow
    float masterslope = ((float)y - (float)oldY) / ((float)x - (float)oldX); // This slope is the slope that we're always checking with.  Eqn of slope is (y2-y1)/(x2-x1
    float currentslope = ((float)y - (float)currentY) / ((float)x - (float)currentX); // This is the slope that is recalculated with every new step.
    Direction directionX; // Variable helps specify the direction that the motor will always be going, there could be a better way, but for now I have specified an individual direction for both the x and y
    Direction directionY;
    AXIS axis;
    int changeofX;
    int changeofY;

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
        std::cout << "suck my dick" << std::endl ;
        return true;
    }

    if (currentX == x) { // For the scenario of the (x,y) resulting in a vertical line
        while (currentY != y) {
            axis = Y;
            stepMotor(axis, directionY);
            currentY = currentY + (-2 * changeofY + 1);
            std::cout << "suck" << std::endl ;
        }
        return true;
    }

    if (currentY == y) { // For the scenario of the (x,y) resulting in a horizontal line
        while (currentX != x) {
            axis = X;
            stepMotor(axis, directionX);
            currentX = currentX + (-2 * changeofX + 1);
            std::cout << "dick" << std::endl ;
        }
        return true;
    }


    while (x != currentX || y != currentY) {

        while (currentslope > (masterslope - precision)) { // I think this is right.  Exits loop when the currentslope decreases past a critical point.  Should specifiy this while loop is for the X increases
            axis = X;
            stepMotor(axis, directionX); // This should make one xs - step towards the desired point.
            currentX = currentX + (-2 * changeofX + 1); // The new currentX location. The "-2*directionX + 1" is the way I can determine whether it increases or decreases. lol its jokes
            currentslope = ((float)y - (float)currentY) / ((float)x - (float)currentX);
            std::cout << "Y: " << y <<" "<< currentY << std::endl;
            std::cout << "X: " << x <<" "<< currentX << std::endl;
            std::cout << "slope: " << currentslope <<" "<< masterslope << std::endl;
            std::cout << "suck this:" << currentslope << ">" << masterslope-precision << std::endl;

        }
        while (currentslope < (masterslope + precision)) { // I think this is right.  Exits loop when the currentslope decreases past a critical point.  Should specifiy this while loop is for the Y increases
            axis = Y;
            stepMotor(axis, directionY); // This should make one Y-step towards the desired point.
            currentY = currentY + (-2 * changeofY + 1); // The new currentY location. The "-2*directionY + 1" is the way I can determine whether it increases or decreases. lol its jokes
            currentslope = ((float)y - (float)currentY) / ((float)x - (float)currentX);
        }

    }


    return true;
}

bool readGPIO(int gpio) {
    //First check the direction of the GPIO:
//    int direction = gpio_get_direction(gpio);
//    if (direction == GPIOF_DIR_OUT) {
//        perror("Can't read GPIO, gpio is set to DIR_OUT");
//        throw std::exception();
//    } else if (direction == GPIOF_DIR_IN) {
    std::cout << "Read GPIO: " << gpio << ", Value: " << gpio_get_value(gpio) << std::endl;
    return (bool) gpio_get_value(gpio);
//    } else {
//        perror("Can't read GPIO, You don't understand GPIOF_DIR_OUT vs GPIOF_DIR_IN");
//        throw std::exception();
//    }
}

int main(const int argc, const char *const argv[]) {
    int i;
    //
    requestGPIOAndSetDirectionOutput(X_AXIS_DIRECTION_GPIO);
    requestGPIOAndSetDirectionOutput(X_AXIS_STEP_GPIO);
    requestGPIOAndSetDirectionOutput(Y_AXIS_DIRECTION_GPIO);
    requestGPIOAndSetDirectionOutput(Y_AXIS_STEP_GPIO);

    requestGPIOAndSetDirectionInput(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO);
    requestGPIOAndSetDirectionInput(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO);
    requestGPIOAndSetDirectionInput(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO);
    requestGPIOAndSetDirectionInput(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO);



    //Accept inputs for testing purposes:
    Direction direction;
    AXIS axis;

    int numSteps = 0;

//    if (!(strcmp(argv[1], "CCW"))) {
//        direction = CCW;
//    } else if (!(strcmp(argv[1], "CW"))) {
//        direction = CW;
//    } else {
//        direction = CCW;
//    }
//
//    if (!(strcmp(argv[2], "X"))) {
//        axis = X;
//    } else if (!(strcmp(argv[2], "Y"))) {
//        axis = Y;
//    } else {
//        axis = X;
//    }

//    // Set the gpio from positive to negative 20 times
//    printf("> begin stepping the motor!\n");
//    for (i = 0; i < 10000; i++) {
//        if (stepMotor(axis, direction)) {
//            numSteps++;
//        } else {
//            std::cout << "numSteps: " << numSteps << std::endl;
//        }
//
//    }

    int x = atoi(argv[1]);
    int y = atoi(argv[2]);

    //////////////////////////////////////////////////////
    //Goto testing:
    gotoXYpoint(x, y);


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

//TODO: Limit switches
//TODO: configure input gpios, for limit switches
//TODO: figure out how you're gonna interface with servo motors.
//TODO: motors go backwards to how they're supposed to.