#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdexcept>

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
void stepMotor(AXIS axis, Direction direction);

void requestGPIOAndSetDirectionOutput(int gpio);

void freeGPIO(int gpio);

//enum AXIS;
//enum Direction;


/////////////////////////////////////////////////////
// Global Variables:

const int X_AXIS_DIRECTION_GPIO = 0;
const int X_AXIS_STEP_GPIO = 1;
const int Y_AXIS_DIRECTION_GPIO = 2;
const int Y_AXIS_STEP_GPIO = 3;
const int STEP_TIME = 10; //time it takes to step a stepper motor in milliseconds.

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

void stepMotor(AXIS axis, Direction direction) {

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
            //set directionGPIO to GND, because GND is CW.
            gpio_set_value(dirGPIO, 0);
            break;
        case CCW:
            //set directionGPIO to HIGH, because HIGH is CCW.
            gpio_set_value(dirGPIO, 1);
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

void freeGPIO(int gpio) {
    if (gpio_free(gpio) < 0) {
        perror("freeGPIO had an error.\n");
    }
}

int main(int argc, char **argv, char **envp) {
    int i;
    //
    requestGPIOAndSetDirectionOutput(X_AXIS_DIRECTION_GPIO);
    requestGPIOAndSetDirectionOutput(X_AXIS_STEP_GPIO);
    requestGPIOAndSetDirectionOutput(Y_AXIS_DIRECTION_GPIO);
    requestGPIOAndSetDirectionOutput(Y_AXIS_STEP_GPIO);

    // Set the gpio from positive to negative 20 times
    printf("> begin stepping the motor!\n");
    for (i = 0; i < 10000; i++) {
//        // set the gpio
//        gpio_set_value(gpio, 1);
////        value = gpio_get_value(gpio);
//        printf("  > Write to GPIO: value\n");
//
//
//        // pause between each read
//        usleep(10);
//        gpio_set_value(gpio, 0);
//        usleep(10);

        stepMotor(X, CW);
    }

    freeGPIO(X_AXIS_DIRECTION_GPIO);
    freeGPIO(X_AXIS_STEP_GPIO);
    freeGPIO(Y_AXIS_DIRECTION_GPIO);
    freeGPIO(Y_AXIS_STEP_GPIO);


    return 0;
}