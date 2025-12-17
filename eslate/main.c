#include <stdio.h>
#include <stdlib.h>
#include "eSlate.h"
#include "../drawing/drawing.h"
#include "../hwr/hwr.h"
#include "../shapes/shapes.h"
#include "../gui/gui.h"
#include "../storage/storage.h"

/**
 * @brief Main application entry point
 *
 * Advanced eSlate application with handwriting recognition,
 * shape correction, GUI, and storage capabilities
 */
int main(void)
{
    printf("eSlate Advanced Drawing Application\n");
    printf("===================================\n\n");

    // Initialize all systems
    esp_err_t ret = eslate_init();
    if (ret != ESP_OK) {
        printf("Failed to initialize eSlate: %d\n", ret);
        return EXIT_FAILURE;
    }

    // Initialize LCD display
    ret = lcd_init_display();
    if (ret != ESP_OK) {
        printf("Failed to initialize LCD display: %d\n", ret);
        return EXIT_FAILURE;
    }

    // Initialize all modules
    drawing_init();
    hwr_init();
    shapes_init();
    gui_init();
    storage_init();

    printf("\nInitialization complete!\n");
    printf("LCD display and capacitive touchscreen are ready.\n\n");

    // Set display backlight to ON
    lcd_set_backlight(1);

    // Calibrate touchscreen for 320x240 display
    ret = touch_calibrate(320, 240);
    if (ret != ESP_OK) {
        printf("Touch calibration failed: %d\n", ret);
        return EXIT_FAILURE;
    }

    // Set touch sensitivity (medium sensitivity)
    ret = touch_set_sensitivity(128);
    if (ret != ESP_OK) {
        printf("Failed to set touch sensitivity: %d\n", ret);
        return EXIT_FAILURE;
    }

    // Clear display and render initial GUI
    extern void lcd_clear(uint16_t color);
    lcd_clear(COLOR_WHITE);
    gui_render();
    drawing_render_canvas();

    printf("\nAdvanced eSlate Features:\n");
    printf("- Drawing tools: Pencil, Eraser, Shapes\n");
    printf("- Handwriting recognition\n");
    printf("- Shape correction\n");
    printf("- Save/Load to flash storage\n");
    printf("- GUI with tool selection\n");
    printf("\nTouch the screen to draw!\n");
    printf("Press Ctrl+C to exit\n\n");

    // Main application loop
    while (1) {
        touch_event_data_t event;

        // Wait for touch event with 50ms timeout
        ret = touch_get_event(&event, 50);

        if (ret == ESP_OK) {
            // First check if GUI handled the event
            if (!gui_handle_touch(event.point.x, event.point.y, event.event)) {
                // GUI didn't handle it, so it's a canvas touch
                switch (event.event) {
                    case TOUCH_EVENT_DOWN:
                        drawing_start_stroke(event.point.x, event.point.y - CANVAS_START_Y, event.point.pressure);
                        break;

                    case TOUCH_EVENT_MOVE:
                        drawing_continue_stroke(event.point.x, event.point.y - CANVAS_START_Y, event.point.pressure);
                        break;

                    case TOUCH_EVENT_UP:
                        drawing_end_stroke();
                        // Re-render canvas after stroke completion
                        drawing_render_canvas();
                        break;

                    default:
                        break;
                }
            }

            // Re-render GUI if needed
            gui_render();
        }
        // If timeout, continue loop
    }

    return EXIT_SUCCESS;
}