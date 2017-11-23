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

/////////////////////////////////////////////////////
// Global Variables:

const int X_AXIS_DIRECTION_GPIO = 0;
const int X_AXIS_STEP_GPIO = 1;
const int Y_AXIS_DIRECTION_GPIO = 2;
const int Y_AXIS_STEP_GPIO = 3;
const int X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO = 6;
const int X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO = 7;
const int Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO = 8;
const int Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO = 9;

const int STEP_TIME = 10000; //time it takes to step a stepper motor in microseconds.

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

//            //Check to see if the limit switch is yelling at you:
//            //Gotta determine if it's the X or the Y axis first though.
//            //IF Y and if CW, then use Limit Y MAX.
//            if (stepGPIO == Y_AXIS_STEP_GPIO && readGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO)) {
//                std::cout << "Failed to step motor, Read Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO Limit switch true." << std::endl;
//                if(readGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO) == -1) {
//                    perror("Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
//                    throw std::exception();
//                }
//                return -1;
//            }
//            //IF X and CW, then use Limit X MAX.
//            if (stepGPIO == X_AXIS_STEP_GPIO && readGPIO(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO)) {
//                std::cout << "Failed to step motor, X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO Read Limit switch true." << std::endl;
//                if(readGPIO(X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO) == -1) {
//                    perror("X_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
//                    throw std::exception();
//                }
//                return -1;
//            }

            //set directionGPIO to HIGH, because GND is CW.
            gpio_set_value(dirGPIO, 1);
            break;
        case CCW:

//            //Check to see if the limit switch is yelling at you:
//            //Gotta determine if it's the X or the Y axis first though.
//            //IF Y and if CCW, then use Limit Y MIN.
//            if (stepGPIO == Y_AXIS_STEP_GPIO && readGPIO(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO)) {
//                std::cout << "Failed to step motor, Read Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO Limit switch true." << std::endl;
//                if(readGPIO(Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO) == -1) {
//                    perror("Y_AXIS_MINIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
//                    throw std::exception();
//                }
//                return -1;
//            }
//            //IF X and CCW, then use Limit X MIN.
//            if (stepGPIO == X_AXIS_STEP_GPIO && readGPIO(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO)) {
//                std::cout << "Failed to step motor, Read X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO Limit switch true." << std::endl;
//                if(readGPIO(X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO) == -1) {
//                    perror("X_AXIS_MINIMUM_LIMIT_SWITCH_GPIO NOT DEFINED");
//                    throw std::exception();
//                }
//                return -1;
//            }

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

    // Set the gpio from positive to negative 20 times
    printf("> begin stepping the motor!\n");
    for (i = 0; i < 10000; i++) {
//        readGPIO(Y_AXIS_MAXIMUM_LIMIT_SWITCH_GPIO);
//        usleep(100 * 1000); // 100 milliseconds.
        stepMotor(Y, CW);
    }

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