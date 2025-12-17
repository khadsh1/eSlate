#include "drawing.h"
#include <string.h>
#include <stdio.h>
#include "../eslate/eSlate.h"

// Global drawing state
static drawing_canvas_t canvas;
static drawing_state_t draw_state;

// Color definitions (RGB565)
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_GRAY      0x8410

/**
 * @brief Initialize drawing system
 */
void drawing_init(void)
{
    // Initialize canvas
    memset(&canvas, 0, sizeof(drawing_canvas_t));
    canvas.background_color = COLOR_WHITE;
    canvas.modified = false;

    // Initialize drawing state
    draw_state.current_tool = TOOL_PENCIL;
    draw_state.current_width = 2;
    draw_state.current_color = COLOR_BLACK;
    draw_state.drawing_active = false;
    memset(&draw_state.current_stroke, 0, sizeof(stroke_t));

    printf("Drawing system initialized\n");
}

/**
 * @brief Create new canvas
 */
void drawing_new_canvas(void)
{
    memset(&canvas, 0, sizeof(drawing_canvas_t));
    canvas.background_color = COLOR_WHITE;
    canvas.modified = true;
    printf("New canvas created\n");
}

/**
 * @brief Start a new stroke at given position
 */
void drawing_start_stroke(uint16_t x, uint16_t y, uint16_t pressure)
{
    if (!drawing_is_point_in_canvas(x, y)) {
        return;
    }

    if (canvas.stroke_count >= MAX_STROKES) {
        printf("Maximum strokes reached\n");
        return;
    }

    draw_state.drawing_active = true;

    // Initialize new stroke
    memset(&draw_state.current_stroke, 0, sizeof(stroke_t));
    draw_state.current_stroke.tool = draw_state.current_tool;
    draw_state.current_stroke.width = draw_state.current_width;
    draw_state.current_stroke.color = draw_state.current_color;

    // Add first point
    draw_state.current_stroke.points[0].x = x;
    draw_state.current_stroke.points[0].y = y;
    draw_state.current_stroke.points[0].pressure = pressure;
    draw_state.current_stroke.point_count = 1;

    printf("Started stroke at (%d, %d)\n", x, y);
}

/**
 * @brief Continue current stroke with new point
 */
void drawing_continue_stroke(uint16_t x, uint16_t y, uint16_t pressure)
{
    if (!draw_state.drawing_active) {
        return;
    }

    if (!drawing_is_point_in_canvas(x, y)) {
        return;
    }

    if (draw_state.current_stroke.point_count >= MAX_POINTS_PER_STROKE) {
        printf("Maximum points per stroke reached\n");
        return;
    }

    // Add point to current stroke
    uint16_t idx = draw_state.current_stroke.point_count;
    draw_state.current_stroke.points[idx].x = x;
    draw_state.current_stroke.points[idx].y = y;
    draw_state.current_stroke.points[idx].pressure = pressure;
    draw_state.current_stroke.point_count++;
}

/**
 * @brief End current stroke
 */
void drawing_end_stroke(void)
{
    if (!draw_state.drawing_active) {
        return;
    }

    if (draw_state.current_stroke.point_count > 0) {
        // Add stroke to canvas
        canvas.strokes[canvas.stroke_count] = draw_state.current_stroke;
        canvas.stroke_count++;
        canvas.modified = true;

        printf("Ended stroke with %d points\n", draw_state.current_stroke.point_count);
    }

    draw_state.drawing_active = false;
    memset(&draw_state.current_stroke, 0, sizeof(stroke_t));
}

/**
 * @brief Set current drawing tool
 */
void drawing_set_tool(drawing_tool_t tool)
{
    draw_state.current_tool = tool;
    printf("Drawing tool set to %d\n", tool);
}

/**
 * @brief Set current stroke width
 */
void drawing_set_width(uint16_t width)
{
    if (width > 0 && width <= 20) {
        draw_state.current_width = width;
        printf("Stroke width set to %d\n", width);
    }
}

/**
 * @brief Set current drawing color
 */
void drawing_set_color(uint16_t color)
{
    draw_state.current_color = color;
    printf("Drawing color set to 0x%04X\n", color);
}

/**
 * @brief Get current drawing state
 */
drawing_state_t* drawing_get_state(void)
{
    return &draw_state;
}

/**
 * @brief Get current canvas
 */
drawing_canvas_t* drawing_get_canvas(void)
{
    return &canvas;
}

/**
 * @brief Clear canvas
 */
void drawing_clear_canvas(void)
{
    memset(&canvas, 0, sizeof(drawing_canvas_t));
    canvas.background_color = COLOR_WHITE;
    canvas.modified = true;
    printf("Canvas cleared\n");
}

/**
 * @brief Undo last stroke
 */
void drawing_undo_last_stroke(void)
{
    if (canvas.stroke_count > 0) {
        canvas.stroke_count--;
        canvas.modified = true;
        printf("Last stroke undone\n");
    }
}

/**
 * @brief Draw a line between two points using Bresenham's algorithm
 */
static void draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    // Use the LCD drawing function
    extern void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
    lcd_draw_line(x0, y0, x1, y1, color);
}

/**
 * @brief Render canvas to LCD display
 */
void drawing_render_canvas(void)
{
    // Clear canvas area to background color
    extern void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
    extern void lcd_draw_string(uint16_t x, uint16_t y, const char* str, uint16_t color);

    lcd_fill_rect(0, CANVAS_START_Y, CANVAS_WIDTH, CANVAS_HEIGHT, canvas.background_color);

    printf("Rendering canvas with %d strokes\n", canvas.stroke_count);

    // Render each stroke
    for (uint16_t i = 0; i < canvas.stroke_count; i++) {
        stroke_t* stroke = &canvas.strokes[i];

        if (stroke->point_count < 2) continue;

        // Draw stroke as connected lines
        for (uint16_t j = 0; j < stroke->point_count - 1; j++) {
            uint16_t x0 = stroke->points[j].x;
            uint16_t y0 = stroke->points[j].y;
            uint16_t x1 = stroke->points[j + 1].x;
            uint16_t y1 = stroke->points[j + 1].y;

            draw_line(x0, y0 + CANVAS_START_Y, x1, y1 + CANVAS_START_Y, stroke->color);
        }
    }

    // Display recognized text if available (simplified)
    static char recognized_text[100] = "";
    if (strlen(recognized_text) > 0) {
        lcd_draw_string(10, CANVAS_START_Y + CANVAS_HEIGHT - 20, recognized_text, COLOR_BLACK);
    }
}

/**
 * @brief Check if point is within canvas area
 */
bool drawing_is_point_in_canvas(uint16_t x, uint16_t y)
{
    return (x < CANVAS_WIDTH && y >= CANVAS_START_Y && y < CANVAS_START_Y + CANVAS_HEIGHT);
}