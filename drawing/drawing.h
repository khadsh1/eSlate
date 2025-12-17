#ifndef DRAWING_H
#define DRAWING_H

#include <stdint.h>
#include <stdbool.h>
#include "../eslate/eSlate.h"

// Drawing canvas dimensions
#define CANVAS_WIDTH    320
#define CANVAS_HEIGHT   200  // Leave space for GUI at bottom
#define CANVAS_START_Y  40   // GUI area at top

// Maximum number of strokes per drawing
#define MAX_STROKES     1000
#define MAX_POINTS_PER_STROKE  1000

// Drawing tools
typedef enum {
    TOOL_PENCIL = 0,
    TOOL_ERASER = 1,
    TOOL_SHAPE_RECT = 2,
    TOOL_SHAPE_CIRCLE = 3,
    TOOL_SHAPE_LINE = 4,
} drawing_tool_t;

// Stroke data structure
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t pressure;
} stroke_point_t;

typedef struct {
    stroke_point_t points[MAX_POINTS_PER_STROKE];
    uint16_t point_count;
    uint16_t width;        // Stroke width
    uint16_t color;        // 16-bit RGB565 color
    drawing_tool_t tool;
} stroke_t;

// Drawing canvas structure
typedef struct {
    stroke_t strokes[MAX_STROKES];
    uint16_t stroke_count;
    uint16_t background_color;
    bool modified;
} drawing_canvas_t;

// Current drawing state
typedef struct {
    drawing_tool_t current_tool;
    uint16_t current_width;
    uint16_t current_color;
    bool drawing_active;
    stroke_t current_stroke;
} drawing_state_t;

/**
 * @brief Initialize drawing system
 */
void drawing_init(void);

/**
 * @brief Create new canvas
 */
void drawing_new_canvas(void);

/**
 * @brief Start a new stroke at given position
 */
void drawing_start_stroke(uint16_t x, uint16_t y, uint16_t pressure);

/**
 * @brief Continue current stroke with new point
 */
void drawing_continue_stroke(uint16_t x, uint16_t y, uint16_t pressure);

/**
 * @brief End current stroke
 */
void drawing_end_stroke(void);

/**
 * @brief Set current drawing tool
 */
void drawing_set_tool(drawing_tool_t tool);

/**
 * @brief Set current stroke width
 */
void drawing_set_width(uint16_t width);

/**
 * @brief Set current drawing color
 */
void drawing_set_color(uint16_t color);

/**
 * @brief Get current drawing state
 */
drawing_state_t* drawing_get_state(void);

/**
 * @brief Get current canvas
 */
drawing_canvas_t* drawing_get_canvas(void);

/**
 * @brief Clear canvas
 */
void drawing_clear_canvas(void);

/**
 * @brief Undo last stroke
 */
void drawing_undo_last_stroke(void);

/**
 * @brief Render canvas to LCD display
 */
void drawing_render_canvas(void);

/**
 * @brief Check if point is within canvas area
 */
bool drawing_is_point_in_canvas(uint16_t x, uint16_t y);

#endif // DRAWING_H