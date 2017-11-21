#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>

#include <ugpio/ugpio.h>

/////////////////////////////////////////////////////
// Function Declarations:

struct PolynomialComponent;

PolynomialComponent *stringToPolynomialFunction(const char input[]);

void stepMotor(int gpioDir, int gpioStp, bool clockwise);


/////////////////////////////////////////////////////
// Global Variables:

const int X_AXIS_DIRECTION = 0;
const int X_AXIS_STEP = 1;
const int Y_AXIS_DIRECTION = 2;
const int Y_AXIS_STEP = 3;

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

/**
 * Steps the motor a single 1.8 degree step
 * @param clockwise A boolean value that determines if you want the motor to go clockwise or not.
 * @param gpioDir The gpio pin your motor direction is attached to.
 * @param gpioStp The gpio pin your motor stp is attached to.
 */
void stepMotor(int gpioDir, int gpioStp, bool clockwise) {

}

int main(int argc, char **argv, char **envp) {
    int i;
    //
    int gpioRequest;
    int gpioDirection;
    int gpio = 0;

    // check if gpio is already exported
    if ((gpioRequest = gpio_is_requested(gpio)) < 0) {
        perror("gpio_is_requested");
        return EXIT_FAILURE;
    }

    // export the gpio
    if (!gpioRequest) {
        printf("> exporting gpio\n");
        if ((gpioDirection = gpio_request(gpio, NULL)) < 0) {
            perror("gpio_request");
            return EXIT_FAILURE;
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

    // Set the gpio from positive to negative 20 times
    printf("> begin setting the GPIO%d\n", gpio);
    for (i = 0; i < 10000; i++) {
        // set the gpio
        gpio_set_value(gpio, 1);
//        value = gpio_get_value(gpio);
        printf("  > Write to GPIO: value\n");


        // pause between each read
        sleep(1);
        gpio_set_value(gpio, 0);
        sleep(1);
    }


    // unexport the gpio
    if (!gpioRequest) {
        printf("> unexporting gpio\n");
        if (gpio_free(gpio) < 0) {
            perror("gpio_free");
        }
    }

    return 0;
}