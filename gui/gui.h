#ifndef GUI_H
#define GUI_H

#include <stdint.h>
#include <stdbool.h>
#include "../drawing/drawing.h"

// GUI element types
typedef enum {
    GUI_BUTTON = 0,
    GUI_SLIDER = 1,
    GUI_LABEL = 2,
} gui_element_type_t;

// GUI button structure
typedef struct {
    uint16_t x, y;
    uint16_t width, height;
    char label[20];
    bool pressed;
    bool enabled;
    void (*callback)(void);
} gui_button_t;

// GUI slider structure
typedef struct {
    uint16_t x, y;
    uint16_t width, height;
    uint16_t value;
    uint16_t min_value, max_value;
    char label[20];
} gui_slider_t;

// GUI label structure
typedef struct {
    uint16_t x, y;
    char text[50];
    uint16_t color;
} gui_label_t;

// GUI element union
typedef struct {
    gui_element_type_t type;
    union {
        gui_button_t button;
        gui_slider_t slider;
        gui_label_t label;
    } data;
} gui_element_t;

// GUI state
#define MAX_GUI_ELEMENTS 20

typedef struct {
    gui_element_t elements[MAX_GUI_ELEMENTS];
    uint16_t element_count;
    bool needs_redraw;
} gui_state_t;

// Predefined GUI layout
#define GUI_HEIGHT 40
#define BUTTON_WIDTH 60
#define BUTTON_HEIGHT 30
#define BUTTON_SPACING 5

// Button IDs for easy reference
typedef enum {
    BTN_PENCIL = 0,
    BTN_ERASER = 1,
    BTN_RECT = 2,
    BTN_CIRCLE = 3,
    BTN_LINE = 4,
    BTN_CLEAR = 5,
    BTN_UNDO = 6,
    BTN_RECOGNIZE = 7,
    BTN_CORRECT = 8,
    BTN_SAVE = 9,
    BTN_LOAD = 10,
} gui_button_id_t;

/**
 * @brief Initialize GUI system
 */
void gui_init(void);

/**
 * @brief Handle touch event on GUI
 *
 * @param x Touch X coordinate
 * @param y Touch Y coordinate
 * @param event Touch event type
 * @return true if GUI handled the event, false if it should be passed to canvas
 */
bool gui_handle_touch(uint16_t x, uint16_t y, touch_event_t event);

/**
 * @brief Render GUI elements
 */
void gui_render(void);

/**
 * @brief Get current GUI state
 */
gui_state_t* gui_get_state(void);

/**
 * @brief Update GUI element
 */
void gui_update_element(uint16_t index, gui_element_t* element);

/**
 * @brief Add GUI element
 */
bool gui_add_element(gui_element_t* element);

/**
 * @brief Remove GUI element
 */
void gui_remove_element(uint16_t index);

/**
 * @brief Create standard drawing GUI layout
 */
void gui_create_drawing_layout(void);

#endif // GUI_H