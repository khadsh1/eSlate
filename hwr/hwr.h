#ifndef HWR_H
#define HWR_H

#include <stdint.h>
#include <stdbool.h>
#include "../drawing/drawing.h"

// Maximum characters that can be recognized in a single word
#define MAX_WORD_LENGTH     20
#define MAX_CANDIDATES      5

// Character recognition result
typedef struct {
    char character;
    float confidence;
} hwr_candidate_t;

typedef struct {
    hwr_candidate_t candidates[MAX_CANDIDATES];
    uint8_t candidate_count;
    uint16_t x_start;
    uint16_t y_start;
    uint16_t width;
    uint16_t height;
} hwr_result_t;

// Word recognition result
typedef struct {
    char word[MAX_WORD_LENGTH + 1];
    float confidence;
    uint16_t x_start;
    uint16_t y_start;
    uint16_t width;
    uint16_t height;
} word_result_t;

/**
 * @brief Initialize handwriting recognition system
 */
void hwr_init(void);

/**
 * @brief Recognize character from stroke data
 *
 * @param strokes Array of strokes forming the character
 * @param stroke_count Number of strokes
 * @param result Recognition result structure to fill
 * @return true if recognition successful, false otherwise
 */
bool hwr_recognize_character(stroke_t* strokes, uint16_t stroke_count, hwr_result_t* result);

/**
 * @brief Recognize word from multiple characters
 *
 * @param results Array of character recognition results
 * @param result_count Number of character results
 * @param word_result Word recognition result structure to fill
 * @return true if recognition successful, false otherwise
 */
bool hwr_recognize_word(hwr_result_t* results, uint16_t result_count, word_result_t* word_result);

/**
 * @brief Extract character strokes from drawing canvas
 *
 * @param canvas Drawing canvas
 * @param x_start Starting X coordinate of character area
 * @param y_start Starting Y coordinate of character area
 * @param width Width of character area
 * @param height Height of character area
 * @param strokes Output array for extracted strokes
 * @param max_strokes Maximum number of strokes to extract
 * @return Number of strokes extracted
 */
uint16_t hwr_extract_character_strokes(drawing_canvas_t* canvas,
                                      uint16_t x_start, uint16_t y_start,
                                      uint16_t width, uint16_t height,
                                      stroke_t* strokes, uint16_t max_strokes);

/**
 * @brief Segment drawing into individual characters
 *
 * @param canvas Drawing canvas
 * @param results Output array for character recognition results
 * @param max_results Maximum number of results
 * @return Number of characters recognized
 */
uint16_t hwr_segment_and_recognize(drawing_canvas_t* canvas,
                                  hwr_result_t* results, uint16_t max_results);

#endif // HWR_H