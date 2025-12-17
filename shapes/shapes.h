#ifndef SHAPES_H
#define SHAPES_H

#include <stdint.h>
#include <stdbool.h>
#include "../drawing/drawing.h"

// Shape types
typedef enum {
    SHAPE_NONE = 0,
    SHAPE_LINE = 1,
    SHAPE_RECTANGLE = 2,
    SHAPE_CIRCLE = 3,
    SHAPE_TRIANGLE = 4,
    SHAPE_POLYGON = 5,
} shape_type_t;

// Shape detection result
typedef struct {
    shape_type_t type;
    float confidence;
    uint16_t x_min, y_min;
    uint16_t x_max, y_max;
    uint16_t center_x, center_y;
    uint16_t width, height;
    uint16_t radius;  // For circles
} shape_result_t;

// Shape correction data
typedef struct {
    shape_type_t type;
    uint16_t points[20];  // x,y pairs for corrected shape
    uint16_t point_count;
    uint16_t color;
    uint16_t width;
} corrected_shape_t;

/**
 * @brief Initialize shape recognition system
 */
void shapes_init(void);

/**
 * @brief Detect shape from stroke data
 *
 * @param strokes Array of strokes forming the shape
 * @param stroke_count Number of strokes
 * @param result Shape detection result structure to fill
 * @return true if shape detected, false otherwise
 */
bool shapes_detect(stroke_t* strokes, uint16_t stroke_count, shape_result_t* result);

/**
 * @brief Generate corrected version of detected shape
 *
 * @param result Shape detection result
 * @param corrected Corrected shape structure to fill
 * @return true if correction successful, false otherwise
 */
bool shapes_correct(shape_result_t* result, corrected_shape_t* corrected);

/**
 * @brief Analyze entire canvas for shapes
 *
 * @param canvas Drawing canvas
 * @param results Output array for shape detection results
 * @param max_results Maximum number of results
 * @return Number of shapes detected
 */
uint16_t shapes_analyze_canvas(drawing_canvas_t* canvas,
                              shape_result_t* results, uint16_t max_results);

/**
 * @brief Apply shape corrections to canvas
 *
 * @param canvas Drawing canvas to modify
 * @param corrections Array of corrected shapes to apply
 * @param correction_count Number of corrections
 */
void shapes_apply_corrections(drawing_canvas_t* canvas,
                             corrected_shape_t* corrections, uint16_t correction_count);

#endif // SHAPES_H