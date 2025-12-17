#include "shapes.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Define PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper functions for shape analysis
static float calculate_distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    int16_t dx = (int16_t)x2 - (int16_t)x1;
    int16_t dy = (int16_t)y2 - (int16_t)y1;
    return sqrtf(dx * dx + dy * dy);
}

static float calculate_angle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    int16_t dx = (int16_t)x2 - (int16_t)x1;
    int16_t dy = (int16_t)y2 - (int16_t)y1;
    return atan2f(dy, dx);
}

static void get_bounding_box(stroke_t* strokes, uint16_t stroke_count,
                           uint16_t* x_min, uint16_t* y_min,
                           uint16_t* x_max, uint16_t* y_max)
{
    *x_min = UINT16_MAX;
    *y_min = UINT16_MAX;
    *x_max = 0;
    *y_max = 0;

    for (uint16_t i = 0; i < stroke_count; i++) {
        stroke_t* stroke = &strokes[i];
        for (uint16_t j = 0; j < stroke->point_count; j++) {
            uint16_t x = stroke->points[j].x;
            uint16_t y = stroke->points[j].y;

            if (x < *x_min) *x_min = x;
            if (y < *y_min) *y_min = y;
            if (x > *x_max) *x_max = x;
            if (y > *y_max) *y_max = y;
        }
    }
}

/**
 * @brief Detect if strokes form a line
 */
static float detect_line(stroke_t* strokes, uint16_t stroke_count, shape_result_t* result)
{
    if (stroke_count != 1) return 0.0f;

    stroke_t* stroke = &strokes[0];
    if (stroke->point_count < 2) return 0.0f;

    // Calculate line parameters using least squares
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    uint16_t n = stroke->point_count;

    for (uint16_t i = 0; i < n; i++) {
        float x = stroke->points[i].x;
        float y = stroke->points[i].y;
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    float slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    float intercept = (sum_y - slope * sum_x) / n;

    // Calculate RMS error
    float error_sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        float x = stroke->points[i].x;
        float y = stroke->points[i].y;
        float predicted_y = slope * x + intercept;
        float error = y - predicted_y;
        error_sum += error * error;
    }
    float rms_error = sqrtf(error_sum / n);

    // Lower RMS error indicates better line fit
    float confidence = 1.0f / (1.0f + rms_error / 10.0f);

    if (confidence > 0.7f) {
        get_bounding_box(strokes, stroke_count, &result->x_min, &result->y_min,
                        &result->x_max, &result->y_max);
        result->type = SHAPE_LINE;
        result->confidence = confidence;
        return confidence;
    }

    return 0.0f;
}

/**
 * @brief Detect if strokes form a rectangle
 */
static float detect_rectangle(stroke_t* strokes, uint16_t stroke_count, shape_result_t* result)
{
    if (stroke_count != 1 && stroke_count != 4) return 0.0f;

    uint16_t x_min, y_min, x_max, y_max;
    get_bounding_box(strokes, stroke_count, &x_min, &y_min, &x_max, &y_max);

    uint16_t width = x_max - x_min;
    uint16_t height = y_max - y_min;

    // Check aspect ratio (not too extreme)
    float aspect_ratio = (float)width / height;
    if (aspect_ratio < 0.2f || aspect_ratio > 5.0f) return 0.0f;

    // For single stroke rectangles, check if it's closed
    if (stroke_count == 1) {
        stroke_t* stroke = &strokes[0];
        if (stroke->point_count < 4) return 0.0f;

        // Check if start and end points are close (closed shape)
        float start_end_dist = calculate_distance(
            stroke->points[0].x, stroke->points[0].y,
            stroke->points[stroke->point_count-1].x, stroke->points[stroke->point_count-1].y
        );

        if (start_end_dist > 20) return 0.0f;  // Not closed

        // Check for rectangular shape by analyzing corner angles
        float confidence = 0.8f;  // Basic confidence for closed shapes
        result->type = SHAPE_RECTANGLE;
        result->confidence = confidence;
        result->x_min = x_min;
        result->y_min = y_min;
        result->x_max = x_max;
        result->y_max = y_max;
        result->width = width;
        result->height = height;
        return confidence;
    }

    // For 4-stroke rectangles, check if they form a rectangle
    if (stroke_count == 4) {
        // This would require more complex analysis of 4 separate strokes
        // For now, assume it's a rectangle if we have 4 strokes in rectangular area
        result->type = SHAPE_RECTANGLE;
        result->confidence = 0.6f;
        result->x_min = x_min;
        result->y_min = y_min;
        result->x_max = x_max;
        result->y_max = y_max;
        result->width = width;
        result->height = height;
        return 0.6f;
    }

    return 0.0f;
}

/**
 * @brief Detect if strokes form a circle
 */
static float detect_circle(stroke_t* strokes, uint16_t stroke_count, shape_result_t* result)
{
    if (stroke_count != 1) return 0.0f;

    stroke_t* stroke = &strokes[0];
    if (stroke->point_count < 8) return 0.0f;  // Need enough points for circle detection

    uint16_t x_min, y_min, x_max, y_max;
    get_bounding_box(strokes, stroke_count, &x_min, &y_min, &x_max, &y_max);

    uint16_t width = x_max - x_min;
    uint16_t height = y_max - y_min;

    // Check if bounding box is roughly square
    float aspect_ratio = (float)width / height;
    if (aspect_ratio < 0.7f || aspect_ratio > 1.4f) return 0.0f;

    // Calculate center
    uint16_t center_x = (x_min + x_max) / 2;
    uint16_t center_y = (y_min + y_max) / 2;
    uint16_t radius = (width + height) / 4;

    // Check if points are approximately on circle circumference
    float total_error = 0;
    uint16_t valid_points = 0;

    for (uint16_t i = 0; i < stroke->point_count; i++) {
        float dist = calculate_distance(center_x, center_y,
                                      stroke->points[i].x, stroke->points[i].y);
        float error = fabsf(dist - radius);
        if (error < radius * 0.3f) {  // Allow 30% tolerance
            total_error += error;
            valid_points++;
        }
    }

    if (valid_points < stroke->point_count * 0.7f) return 0.0f;  // Not enough points on circle

    float avg_error = total_error / valid_points;
    float confidence = 1.0f / (1.0f + avg_error / 5.0f);

    if (confidence > 0.6f) {
        result->type = SHAPE_CIRCLE;
        result->confidence = confidence;
        result->x_min = x_min;
        result->y_min = y_min;
        result->x_max = x_max;
        result->y_max = y_max;
        result->center_x = center_x;
        result->center_y = center_y;
        result->radius = radius;
        return confidence;
    }

    return 0.0f;
}

/**
 * @brief Initialize shape recognition system
 */
void shapes_init(void)
{
    printf("Shape recognition system initialized\n");
}

/**
 * @brief Detect shape from stroke data
 */
bool shapes_detect(stroke_t* strokes, uint16_t stroke_count, shape_result_t* result)
{
    if (!result || stroke_count == 0) return false;

    memset(result, 0, sizeof(shape_result_t));

    // Try different shape detectors
    float line_conf = detect_line(strokes, stroke_count, result);
    float rect_conf = detect_rectangle(strokes, stroke_count, result);
    float circle_conf = detect_circle(strokes, stroke_count, result);

    // Return the best match
    if (line_conf >= rect_conf && line_conf >= circle_conf && line_conf > 0.5f) {
        result->type = SHAPE_LINE;
        result->confidence = line_conf;
        return true;
    } else if (rect_conf >= line_conf && rect_conf >= circle_conf && rect_conf > 0.5f) {
        result->type = SHAPE_RECTANGLE;
        result->confidence = rect_conf;
        return true;
    } else if (circle_conf >= line_conf && circle_conf >= rect_conf && circle_conf > 0.5f) {
        result->type = SHAPE_CIRCLE;
        result->confidence = circle_conf;
        return true;
    }

    return false;
}

/**
 * @brief Generate corrected version of detected shape
 */
bool shapes_correct(shape_result_t* result, corrected_shape_t* corrected)
{
    if (!result || !corrected) return false;

    corrected->type = result->type;
    corrected->color = 0xF800;  // Red color for corrected shapes
    corrected->width = 2;

    switch (result->type) {
        case SHAPE_LINE:
            // Create perfect horizontal/vertical or diagonal line
            corrected->point_count = 2;
            corrected->points[0] = result->x_min;
            corrected->points[1] = result->y_min;
            corrected->points[2] = result->x_max;
            corrected->points[3] = result->y_max;
            break;

        case SHAPE_RECTANGLE:
            // Create perfect rectangle
            corrected->point_count = 4;
            corrected->points[0] = result->x_min;  // top-left
            corrected->points[1] = result->y_min;
            corrected->points[2] = result->x_max;  // top-right
            corrected->points[3] = result->y_min;
            corrected->points[4] = result->x_max;  // bottom-right
            corrected->points[5] = result->y_max;
            corrected->points[6] = result->x_min;  // bottom-left
            corrected->points[7] = result->y_max;
            break;

        case SHAPE_CIRCLE:
            // Create perfect circle (approximated with points)
            corrected->point_count = 16;  // 16-point circle approximation
            for (uint16_t i = 0; i < 16; i++) {
                float angle = 2 * M_PI * i / 16;
                corrected->points[i*2] = result->center_x + (uint16_t)(result->radius * cosf(angle));
                corrected->points[i*2+1] = result->center_y + (uint16_t)(result->radius * sinf(angle));
            }
            break;

        default:
            return false;
    }

    return true;
}

/**
 * @brief Analyze entire canvas for shapes
 */
uint16_t shapes_analyze_canvas(drawing_canvas_t* canvas,
                              shape_result_t* results, uint16_t max_results)
{
    uint16_t detected = 0;

    // Group strokes into potential shapes (simplified approach)
    // In a real implementation, this would use more sophisticated clustering

    for (uint16_t i = 0; i < canvas->stroke_count && detected < max_results; i++) {
        // For now, treat each stroke as a potential shape
        stroke_t* strokes = &canvas->strokes[i];
        shape_result_t* result = &results[detected];

        if (shapes_detect(strokes, 1, result)) {
            detected++;
        }
    }

    return detected;
}

/**
 * @brief Apply shape corrections to canvas
 */
void shapes_apply_corrections(drawing_canvas_t* canvas,
                             corrected_shape_t* corrections, uint16_t correction_count)
{
    for (uint16_t i = 0; i < correction_count; i++) {
        corrected_shape_t* corr = &corrections[i];

        if (canvas->stroke_count >= MAX_STROKES) break;

        // Create a new stroke for the corrected shape
        stroke_t* new_stroke = &canvas->strokes[canvas->stroke_count];
        memset(new_stroke, 0, sizeof(stroke_t));

        new_stroke->tool = TOOL_PENCIL;
        new_stroke->width = corr->width;
        new_stroke->color = corr->color;

        // Convert corrected shape points to stroke points
        uint16_t max_points = corr->point_count;
        if (max_points > MAX_POINTS_PER_STROKE) max_points = MAX_POINTS_PER_STROKE;

        for (uint16_t j = 0; j < max_points; j++) {
            new_stroke->points[j].x = corr->points[j*2];
            new_stroke->points[j].y = corr->points[j*2+1];
            new_stroke->points[j].pressure = 255;
        }
        new_stroke->point_count = max_points;

        canvas->stroke_count++;
        canvas->modified = true;
    }

    printf("Applied %d shape corrections\n", correction_count);
}