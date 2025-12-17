#include "hwr.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Character templates (simplified stroke-based recognition)
// Each character is defined by number of strokes and basic geometric features
typedef struct {
    char character;
    uint8_t stroke_count;
    uint8_t features[8];  // Feature vector for recognition
} char_template_t;

// Basic character templates (A-Z, a-z, 0-9)
static const char_template_t char_templates[] = {
    // Letters A-Z
    {'A', 2, {1, 0, 1, 1, 0, 0, 0, 0}},  // Two strokes, crossing
    {'B', 2, {1, 1, 0, 0, 1, 0, 0, 0}},  // Two strokes, vertical with curves
    {'C', 1, {0, 1, 0, 0, 0, 1, 0, 0}},  // Single curve
    {'D', 2, {1, 1, 0, 0, 0, 1, 0, 0}},  // Vertical with curve
    {'E', 4, {1, 0, 0, 0, 1, 0, 0, 0}},  // Four strokes, horizontal lines
    {'F', 3, {1, 0, 0, 0, 1, 0, 0, 0}},  // Three strokes
    {'G', 1, {0, 1, 0, 0, 0, 1, 1, 0}},  // Single curve with line
    {'H', 3, {1, 0, 1, 0, 1, 0, 0, 0}},  // Three strokes, vertical with cross
    {'I', 3, {0, 0, 0, 0, 1, 0, 0, 0}},  // Three horizontal strokes
    {'J', 1, {0, 1, 0, 0, 0, 0, 1, 0}},  // Hook shape
    {'K', 3, {1, 0, 1, 1, 0, 0, 0, 0}},  // Vertical with diagonals
    {'L', 2, {1, 0, 0, 0, 0, 0, 1, 0}},  // Vertical with horizontal
    {'M', 1, {1, 0, 1, 1, 0, 0, 0, 0}},  // Single stroke with peaks
    {'N', 1, {1, 0, 1, 1, 0, 0, 0, 0}},  // Single diagonal stroke
    {'O', 1, {0, 1, 0, 0, 0, 0, 0, 1}},  // Single closed curve
    {'P', 2, {1, 1, 0, 0, 1, 0, 0, 0}},  // Vertical with curve
    {'Q', 1, {0, 1, 0, 0, 0, 0, 1, 1}},  // Closed curve with tail
    {'R', 2, {1, 1, 1, 1, 0, 0, 0, 0}},  // Vertical with curve and diagonal
    {'S', 1, {0, 1, 0, 0, 0, 1, 0, 0}},  // Single S-curve
    {'T', 2, {0, 0, 0, 0, 1, 0, 1, 0}},  // Horizontal with vertical
    {'U', 1, {1, 1, 0, 0, 0, 0, 0, 0}},  // Single U-shape
    {'V', 1, {1, 0, 1, 1, 0, 0, 0, 0}},  // Single V-shape
    {'W', 1, {1, 0, 1, 1, 1, 0, 0, 0}},  // Single W-shape
    {'X', 2, {1, 0, 1, 1, 0, 0, 0, 0}},  // Two crossing diagonals
    {'Y', 2, {1, 0, 1, 1, 0, 0, 0, 0}},  // Vertical with diagonal
    {'Z', 3, {0, 0, 1, 1, 0, 0, 0, 0}},  // Three strokes, zigzag

    // Numbers 0-9
    {'0', 1, {0, 1, 0, 0, 0, 0, 0, 1}},  // Closed curve
    {'1', 1, {1, 0, 0, 0, 0, 0, 1, 0}},  // Single vertical
    {'2', 1, {0, 1, 0, 0, 0, 1, 0, 0}},  // Single curve
    {'3', 2, {0, 1, 0, 0, 0, 1, 0, 0}},  // Two curves
    {'4', 3, {1, 0, 1, 0, 1, 0, 0, 0}},  // Three strokes
    {'5', 2, {0, 1, 0, 0, 1, 0, 0, 0}},  // Curve with horizontal
    {'6', 1, {0, 1, 0, 0, 0, 1, 0, 0}},  // Single curve
    {'7', 2, {0, 0, 1, 1, 0, 0, 0, 0}},  // Two strokes
    {'8', 2, {0, 1, 0, 0, 0, 1, 0, 0}},  // Two loops
    {'9', 1, {0, 1, 0, 0, 0, 1, 0, 0}},  // Single curve
};

#define NUM_TEMPLATES (sizeof(char_templates) / sizeof(char_template_t))

/**
 * @brief Initialize handwriting recognition system
 */
void hwr_init(void)
{
    printf("Handwriting recognition system initialized with %lu character templates\n", (unsigned long)NUM_TEMPLATES);
}

/**
 * @brief Extract features from stroke data
 */
static void extract_features(stroke_t* strokes, uint16_t stroke_count, uint8_t* features)
{
    memset(features, 0, 8);

    if (stroke_count == 0) return;

    // Feature 0: Has vertical strokes
    // Feature 1: Has curved strokes
    // Feature 2: Has horizontal strokes
    // Feature 3: Has diagonal strokes
    // Feature 4: Has crossing strokes
    // Feature 5: Has closed shapes
    // Feature 6: Has hooks/tails
    // Feature 7: Multiple components

    for (uint16_t i = 0; i < stroke_count; i++) {
        stroke_t* stroke = &strokes[i];

        if (stroke->point_count < 2) continue;

        // Analyze stroke direction and curvature
        int16_t dx = (int16_t)stroke->points[stroke->point_count-1].x - (int16_t)stroke->points[0].x;
        int16_t dy = (int16_t)stroke->points[stroke->point_count-1].y - (int16_t)stroke->points[0].y;

        // Check for vertical strokes
        if (abs(dy) > abs(dx) * 2) {
            features[0] = 1;
        }

        // Check for horizontal strokes
        if (abs(dx) > abs(dy) * 2) {
            features[2] = 1;
        }

        // Check for diagonal strokes
        if (abs(dx) > 10 && abs(dy) > 10) {
            features[3] = 1;
        }

        // Simple curvature detection (change in direction)
        bool has_curve = false;
        for (uint16_t j = 1; j < stroke->point_count - 1; j++) {
            int16_t dx1 = (int16_t)stroke->points[j].x - (int16_t)stroke->points[j-1].x;
            int16_t dy1 = (int16_t)stroke->points[j].y - (int16_t)stroke->points[j-1].y;
            int16_t dx2 = (int16_t)stroke->points[j+1].x - (int16_t)stroke->points[j].x;
            int16_t dy2 = (int16_t)stroke->points[j+1].y - (int16_t)stroke->points[j].y;

            // Check for significant direction change
            if (abs(dx1 * dy2 - dy1 * dx2) > 100) {
                has_curve = true;
                break;
            }
        }
        if (has_curve) {
            features[1] = 1;
        }
    }

    // Check for crossing strokes (simplified)
    if (stroke_count >= 2) {
        features[4] = 1;
    }

    // Check for closed shapes (first and last points close)
    for (uint16_t i = 0; i < stroke_count; i++) {
        stroke_t* stroke = &strokes[i];
        if (stroke->point_count > 3) {
            int16_t dist = abs((int16_t)stroke->points[0].x - (int16_t)stroke->points[stroke->point_count-1].x) +
                          abs((int16_t)stroke->points[0].y - (int16_t)stroke->points[stroke->point_count-1].y);
            if (dist < 20) {
                features[5] = 1;
                break;
            }
        }
    }

    // Multiple strokes indicate complex characters
    if (stroke_count > 1) {
        features[7] = 1;
    }
}

/**
 * @brief Calculate similarity between feature vectors
 */
static float calculate_similarity(const uint8_t* features1, const uint8_t* features2)
{
    int matches = 0;
    int total = 0;

    for (int i = 0; i < 8; i++) {
        if (features1[i] || features2[i]) {
            total++;
            if (features1[i] == features2[i]) {
                matches++;
            }
        }
    }

    return total > 0 ? (float)matches / total : 0.0f;
}

/**
 * @brief Recognize character from stroke data
 */
bool hwr_recognize_character(stroke_t* strokes, uint16_t stroke_count, hwr_result_t* result)
{
    if (!result || stroke_count == 0) {
        return false;
    }

    // Extract features from input strokes
    uint8_t input_features[8];
    extract_features(strokes, stroke_count, input_features);

    // Find best matching templates
    result->candidate_count = 0;

    for (uint16_t i = 0; i < NUM_TEMPLATES; i++) {
        const char_template_t* templ = &char_templates[i];

        // Check stroke count match (with some tolerance)
        if (abs((int)stroke_count - (int)templ->stroke_count) > 1) {
            continue;
        }

        float similarity = calculate_similarity(input_features, templ->features);

        // Add to candidates if similarity is reasonable
        if (similarity > 0.3f && result->candidate_count < MAX_CANDIDATES) {
            result->candidates[result->candidate_count].character = templ->character;
            result->candidates[result->candidate_count].confidence = similarity;
            result->candidate_count++;
        }
    }

    // Sort candidates by confidence (simple bubble sort)
    for (uint8_t i = 0; i < result->candidate_count - 1; i++) {
        for (uint8_t j = 0; j < result->candidate_count - i - 1; j++) {
            if (result->candidates[j].confidence < result->candidates[j + 1].confidence) {
                hwr_candidate_t temp = result->candidates[j];
                result->candidates[j] = result->candidates[j + 1];
                result->candidates[j + 1] = temp;
            }
        }
    }

    return result->candidate_count > 0;
}

/**
 * @brief Recognize word from multiple characters
 */
bool hwr_recognize_word(hwr_result_t* results, uint16_t result_count, word_result_t* word_result)
{
    if (!word_result || result_count == 0 || result_count > MAX_WORD_LENGTH) {
        return false;
    }

    word_result->confidence = 1.0f;
    word_result->word[0] = '\0';

    // Build word from best character candidates
    for (uint16_t i = 0; i < result_count; i++) {
        if (results[i].candidate_count > 0) {
            char c = results[i].candidates[0].character;
            word_result->word[i] = c;
            word_result->confidence *= results[i].candidates[0].confidence;
        } else {
            word_result->word[i] = '?';
            word_result->confidence *= 0.1f;
        }
    }
    word_result->word[result_count] = '\0';

    return true;
}

/**
 * @brief Extract character strokes from drawing canvas
 */
uint16_t hwr_extract_character_strokes(drawing_canvas_t* canvas,
                                      uint16_t x_start, uint16_t y_start,
                                      uint16_t width, uint16_t height,
                                      stroke_t* strokes, uint16_t max_strokes)
{
    uint16_t extracted = 0;

    for (uint16_t i = 0; i < canvas->stroke_count && extracted < max_strokes; i++) {
        stroke_t* stroke = &canvas->strokes[i];

        // Check if stroke intersects with character area
        bool intersects = false;
        for (uint16_t j = 0; j < stroke->point_count; j++) {
            uint16_t px = stroke->points[j].x;
            uint16_t py = stroke->points[j].y;

            if (px >= x_start && px < x_start + width &&
                py >= y_start && py < y_start + height) {
                intersects = true;
                break;
            }
        }

        if (intersects) {
            strokes[extracted] = *stroke;
            extracted++;
        }
    }

    return extracted;
}

/**
 * @brief Segment drawing into individual characters
 */
uint16_t hwr_segment_and_recognize(drawing_canvas_t* canvas,
                                  hwr_result_t* results, uint16_t max_results)
{
    uint16_t recognized = 0;

    // Simple segmentation: divide canvas into character-sized regions
    // This is a very basic implementation - real segmentation would be much more complex
    uint16_t char_width = 40;
    uint16_t char_height = 60;

    for (uint16_t x = 0; x < CANVAS_WIDTH - char_width && recognized < max_results; x += char_width) {
        for (uint16_t y = CANVAS_START_Y; y < CANVAS_START_Y + CANVAS_HEIGHT - char_height; y += char_height) {

            // Extract strokes in this region
            stroke_t char_strokes[MAX_STROKES];
            uint16_t stroke_count = hwr_extract_character_strokes(canvas, x, y, char_width, char_height,
                                                                char_strokes, MAX_STROKES);

            if (stroke_count > 0) {
                // Recognize character
                hwr_result_t* result = &results[recognized];
                if (hwr_recognize_character(char_strokes, stroke_count, result)) {
                    result->x_start = x;
                    result->y_start = y;
                    result->width = char_width;
                    result->height = char_height;
                    recognized++;
                }
            }
        }
    }

    return recognized;
}