#include "gui.h"
#include <string.h>
#include <stdio.h>
#include "../drawing/drawing.h"
#include "../hwr/hwr.h"
#include "../shapes/shapes.h"

// Global GUI state
static gui_state_t gui_state;

// Button callback functions
static void btn_pencil_callback(void) {
    drawing_set_tool(TOOL_PENCIL);
    printf("Selected pencil tool\n");
}

static void btn_eraser_callback(void) {
    drawing_set_tool(TOOL_ERASER);
    printf("Selected eraser tool\n");
}

static void btn_rect_callback(void) {
    drawing_set_tool(TOOL_SHAPE_RECT);
    printf("Selected rectangle tool\n");
}

static void btn_circle_callback(void) {
    drawing_set_tool(TOOL_SHAPE_CIRCLE);
    printf("Selected circle tool\n");
}

static void btn_line_callback(void) {
    drawing_set_tool(TOOL_SHAPE_LINE);
    printf("Selected line tool\n");
}

static void btn_clear_callback(void) {
    drawing_clear_canvas();
    printf("Canvas cleared\n");
}

static void btn_undo_callback(void) {
    drawing_undo_last_stroke();
    printf("Undid last stroke\n");
}

static void btn_recognize_callback(void) {
    // Recognize handwriting on canvas
    hwr_result_t results[MAX_WORD_LENGTH];
    uint16_t recognized = hwr_segment_and_recognize(drawing_get_canvas(), results, MAX_WORD_LENGTH);

    if (recognized > 0) {
        printf("Recognized %d characters:\n", recognized);
        for (uint16_t i = 0; i < recognized; i++) {
            if (results[i].candidate_count > 0) {
                printf("  '%c' (confidence: %.2f)\n",
                       results[i].candidates[0].character,
                       results[i].candidates[0].confidence);
            }
        }
    } else {
        printf("No characters recognized\n");
    }
}

static void btn_correct_callback(void) {
    // Detect and correct shapes on canvas
    shape_result_t shape_results[10];
    uint16_t detected = shapes_analyze_canvas(drawing_get_canvas(), shape_results, 10);

    if (detected > 0) {
        corrected_shape_t corrections[10];
        uint16_t corrections_count = 0;

        for (uint16_t i = 0; i < detected; i++) {
            if (shapes_correct(&shape_results[i], &corrections[corrections_count])) {
                corrections_count++;
            }
        }

        if (corrections_count > 0) {
            shapes_apply_corrections(drawing_get_canvas(), corrections, corrections_count);
            printf("Applied %d shape corrections\n", corrections_count);
        }
    } else {
        printf("No shapes detected for correction\n");
    }
}

static void btn_save_callback(void) {
    printf("Save functionality not yet implemented\n");
    // TODO: Implement save to flash
}

static void btn_load_callback(void) {
    printf("Load functionality not yet implemented\n");
    // TODO: Implement load from flash
}

/**
 * @brief Initialize GUI system
 */
void gui_init(void)
{
    memset(&gui_state, 0, sizeof(gui_state_t));
    gui_state.needs_redraw = true;

    gui_create_drawing_layout();
    printf("GUI system initialized\n");
}

/**
 * @brief Handle touch event on GUI
 */
bool gui_handle_touch(uint16_t x, uint16_t y, touch_event_t event)
{
    // Check if touch is in GUI area (top 40 pixels)
    if (y >= GUI_HEIGHT) {
        return false;  // Pass to canvas
    }

    // Find which GUI element was touched
    for (uint16_t i = 0; i < gui_state.element_count; i++) {
        gui_element_t* elem = &gui_state.elements[i];

        if (elem->type == GUI_BUTTON) {
            gui_button_t* btn = &elem->data.button;

            if (x >= btn->x && x < btn->x + btn->width &&
                y >= btn->y && y < btn->y + btn->height) {

                if (event == TOUCH_EVENT_DOWN) {
                    btn->pressed = true;
                    gui_state.needs_redraw = true;
                } else if (event == TOUCH_EVENT_UP && btn->pressed) {
                    btn->pressed = false;
                    gui_state.needs_redraw = true;

                    // Call button callback
                    if (btn->callback) {
                        btn->callback();
                    }
                }

                return true;  // GUI handled the event
            }
        }
    }

    return false;  // No GUI element touched
}

/**
 * @brief Render GUI elements
 */
void gui_render(void)
{
    if (!gui_state.needs_redraw) {
        return;
    }

    printf("Rendering GUI...\n");

    // Render each element (simplified - would need actual LCD drawing functions)
    for (uint16_t i = 0; i < gui_state.element_count; i++) {
        gui_element_t* elem = &gui_state.elements[i];

        switch (elem->type) {
            case GUI_BUTTON: {
                gui_button_t* btn = &elem->data.button;
                printf("  Button '%s' at (%d,%d) %s\n",
                       btn->label, btn->x, btn->y,
                       btn->pressed ? "[PRESSED]" : "");
                break;
            }
            case GUI_LABEL: {
                gui_label_t* lbl = &elem->data.label;
                printf("  Label '%s' at (%d,%d)\n", lbl->text, lbl->x, lbl->y);
                break;
            }
            default:
                break;
        }
    }

    gui_state.needs_redraw = false;
}

/**
 * @brief Get current GUI state
 */
gui_state_t* gui_get_state(void)
{
    return &gui_state;
}

/**
 * @brief Update GUI element
 */
void gui_update_element(uint16_t index, gui_element_t* element)
{
    if (index < gui_state.element_count) {
        gui_state.elements[index] = *element;
        gui_state.needs_redraw = true;
    }
}

/**
 * @brief Add GUI element
 */
bool gui_add_element(gui_element_t* element)
{
    if (gui_state.element_count >= MAX_GUI_ELEMENTS) {
        return false;
    }

    gui_state.elements[gui_state.element_count] = *element;
    gui_state.element_count++;
    gui_state.needs_redraw = true;
    return true;
}

/**
 * @brief Remove GUI element
 */
void gui_remove_element(uint16_t index)
{
    if (index < gui_state.element_count) {
        for (uint16_t i = index; i < gui_state.element_count - 1; i++) {
            gui_state.elements[i] = gui_state.elements[i + 1];
        }
        gui_state.element_count--;
        gui_state.needs_redraw = true;
    }
}

/**
 * @brief Create standard drawing GUI layout
 */
void gui_create_drawing_layout(void)
{
    // Clear existing elements
    gui_state.element_count = 0;

    // Define button layout
    struct {
        char label[20];
        void (*callback)(void);
    } button_defs[] = {
        {"Pencil", btn_pencil_callback},
        {"Eraser", btn_eraser_callback},
        {"Rect", btn_rect_callback},
        {"Circle", btn_circle_callback},
        {"Line", btn_line_callback},
        {"Clear", btn_clear_callback},
        {"Undo", btn_undo_callback},
        {"Recognize", btn_recognize_callback},
        {"Correct", btn_correct_callback},
        {"Save", btn_save_callback},
        {"Load", btn_load_callback},
    };

    uint16_t num_buttons = sizeof(button_defs) / sizeof(button_defs[0]);
    uint16_t x_pos = 5;

    for (uint16_t i = 0; i < num_buttons; i++) {
        gui_element_t elem;
        memset(&elem, 0, sizeof(gui_element_t));
        elem.type = GUI_BUTTON;

        gui_button_t* btn = &elem.data.button;
        btn->x = x_pos;
        btn->y = 5;
        btn->width = BUTTON_WIDTH;
        btn->height = BUTTON_HEIGHT;
        btn->pressed = false;
        btn->enabled = true;
        strcpy(btn->label, button_defs[i].label);
        btn->callback = button_defs[i].callback;

        gui_add_element(&elem);

        x_pos += BUTTON_WIDTH + BUTTON_SPACING;

        // Wrap to next row if needed
        if (x_pos + BUTTON_WIDTH > CANVAS_WIDTH) {
            x_pos = 5;
            // Would need to adjust y_pos for multiple rows
        }
    }

    // Add tool size slider
    gui_element_t slider_elem;
    memset(&slider_elem, 0, sizeof(gui_element_t));
    slider_elem.type = GUI_SLIDER;

    gui_slider_t* slider = &slider_elem.data.slider;
    slider->x = x_pos;
    slider->y = 5;
    slider->width = 80;
    slider->height = BUTTON_HEIGHT;
    slider->value = 2;
    slider->min_value = 1;
    slider->max_value = 10;
    strcpy(slider->label, "Size");

    gui_add_element(&slider_elem);

    printf("Created GUI layout with %d elements\n", gui_state.element_count);
}