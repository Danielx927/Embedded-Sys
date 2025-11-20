#include "barcode.h"
#include "utils.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"

volatile uint64_t last_edge_us = 0;
volatile bool     last_level   = 0;
volatile uint32_t widths[MAX_WIDTHS];
volatile int      num_widths   = 0;
volatile uint32_t isr_call_count = 0;  // Debug: count ISR calls
char decoded_barcode[10] = {0};


void barcode_init(void) {
    gpio_init(RIGHT_IR_DIGITAL_GPIO);
    gpio_set_dir(RIGHT_IR_DIGITAL_GPIO, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_GPIO);
    gpio_set_irq_enabled(RIGHT_IR_DIGITAL_GPIO, 
                        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

bool_t barcode_check_and_process(uint32_t now_ms, 
                                 turn_dir_t *turn_dir,
                                 uint32_t *turn_start) {
    if (num_widths >= 29 && last_edge_us != 0 && 
        (now_ms - (uint32_t)(last_edge_us / 1000) >= BARCODE_PROCESS_TIMEOUT_MS)) {
        decode_and_process();
        num_widths = 0;
        
        if (strlen(decoded_barcode) > 0) {
            if (strcmp(decoded_barcode, "XGX") == 0 || 
                strcmp(decoded_barcode, "9Q9") == 0) {
                *turn_dir = TURN_LEFT;
                *turn_start = now_ms + TURN_DELAY_MS;
                return true;
            } else if (strcmp(decoded_barcode, "939") == 0) {
                *turn_dir = TURN_RIGHT;
                *turn_start = now_ms + TURN_DELAY_MS;
                return true;
            }
            memset(decoded_barcode, 0, sizeof(decoded_barcode));
        }
    }
    return false;
}

/**
 * @brief Comparator for qsort on uint32_t
 */
int uint32_cmp(const void *a, const void *b) {
    uint32_t va = *(const uint32_t *)a;
    uint32_t vb = *(const uint32_t *)b;
    return (va > vb) - (va < vb);
}

/**
 * @brief GPIO IRQ handler for barcode
 */
void ir_isr(uint gpio, uint32_t events) {
    if (gpio != RIGHT_IR_DIGITAL_GPIO) return;
    isr_call_count++;  // Debug: increment on every ISR call
    uint64_t t_now = micros_now();

    bool level_now = gpio_get(RIGHT_IR_DIGITAL_GPIO);

    if (last_edge_us != 0) {
        uint32_t dt = (uint32_t)(t_now - last_edge_us);
        if (dt >= MIN_EDGE_US && dt < 5000000 && num_widths < MAX_WIDTHS) {
            uint32_t entry = dt | (last_level ? 0x80000000U : 0);  // Bars when GPIO high
            widths[num_widths++] = entry;
        }
    }

    last_edge_us = t_now;
    last_level = level_now;
}

/**
 * @brief Get digit from two wide bar positions
 */
int get_bar_digit(int bar_pos1, int bar_pos2) {
    if (bar_pos1 > bar_pos2) { int temp = bar_pos1; bar_pos1 = bar_pos2; bar_pos2 = temp; }
    if (bar_pos1 == 1 && bar_pos2 == 2) return 1;
    if (bar_pos1 == 1 && bar_pos2 == 3) return 2;
    if (bar_pos1 == 2 && bar_pos2 == 3) return 3;
    if (bar_pos1 == 1 && bar_pos2 == 4) return 4;
    if (bar_pos1 == 2 && bar_pos2 == 4) return 5;
    if (bar_pos1 == 1 && bar_pos2 == 5) return 6;
    if (bar_pos1 == 2 && bar_pos2 == 5) return 7;
    if (bar_pos1 == 3 && bar_pos2 == 5) return 8;
    if (bar_pos1 == 3 && bar_pos2 == 4) return 9;
    if (bar_pos1 == 4 && bar_pos2 == 5) return 0;
    return -1; // Invalid
}

/**
 * @brief Decode a single character from 9 binary elements
 */
char decode_char(const int binary[9]) {
    int b1 = binary[0], s1 = binary[1], b2 = binary[2], s2 = binary[3],
        b3 = binary[4], s3 = binary[5], b4 = binary[6], s4 = binary[7],
        b5 = binary[8];

    int num_wide = b1 + s1 + b2 + s2 + b3 + s3 + b4 + s4 + b5;
    if (num_wide != 3) return '?';

    int wide_bars = b1 + b2 + b3 + b4 + b5;
    int wide_spaces = s1 + s2 + s3 + s4;

    if (wide_bars == 0 && wide_spaces == 3) {
        // Special characters (-, ., space, *).
        int narrow_space_pos = 0;
        if (!s1) narrow_space_pos = 1;
        else if (!s2) narrow_space_pos = 2;
        else if (!s3) narrow_space_pos = 3;
        else if (!s4) narrow_space_pos = 4;

        switch (narrow_space_pos) {
            case 1: return '-';
            case 2: return '.';
            case 3: return ' ';
            case 4: return '*';
            default: return '?';
        }
    } else if (wide_bars == 2 && wide_spaces == 1) {
        // Normal characters (digits, letters).
        // Find wide bar positions (1-based).
        int bar_pos[2] = {0, 0};
        int count = 0;
        if (b1) bar_pos[count++] = 1;
        if (b2) bar_pos[count++] = 2;
        if (b3) bar_pos[count++] = 3;
        if (b4) bar_pos[count++] = 4;
        if (b5) bar_pos[count++] = 5;
        if (count != 2) return '?';

        int digit = get_bar_digit(bar_pos[0], bar_pos[1]);
        if (digit == -1) return '?';

        // Find wide space position (1-based).
        int wide_space_pos = 0;
        if (s1) wide_space_pos = 1;
        else if (s2) wide_space_pos = 2;
        else if (s3) wide_space_pos = 3;
        else if (s4) wide_space_pos = 4;
        if (wide_space_pos == 0) return '?';

        int base = (wide_space_pos - 1) * 10;
        int value = base + digit;
        if (value < 10) {
            return '0' + value;
        } else if (value <= 35) {
            return 'A' + (value - 10);
        } else {
            return '?';
        }
    }
    return '?';
}

/**
 * @brief Reverse map for reversed patterns
 */
char reverse_map(char c) {
    switch (c) {
        case 'V': return '0';
        case 'U': return '1';
        case 'Z': return '5';
        case 'Y': return '7';
        case 'W': return '8';
        case 'X': return '9';
        case 'L': return 'A';
        case 'K': return 'B';
        case 'S': return 'C';
        case 'T': return 'D';
        case 'R': return 'E';
        case 'Q': return 'F';
        case 'O': return 'G';
        case 'P': return 'H';
        case 'N': return 'I';
        case 'M': return 'J';
        case 'I': return 'K';
        case 'J': return 'L';
        case 'F': return 'M';
        case 'E': return 'N';
        case 'G': return 'O';
        case 'H': return 'P';
        case 'B': return 'Q';
        case 'A': return 'R';
        case 'C': return 'S';
        case 'D': return 'T';
        case '1': return 'U';
        case '0': return 'V';
        case '4': return 'W';
        case '3': return 'X';
        case '5': return 'Y';
        case '2': return 'Z';
        default: return '?';
    }
}

/**
 * @brief Reverse a string in place
 */
void str_reverse(char *str) {
    int len = strlen(str);
    for (int i = 0; i < len / 2; i++) {
        char temp = str[i];
        str[i] = str[len - 1 - i];
        str[len - 1 - i] = temp;
    }
}

/**
 * @brief Find threshold using sorted widths
 */
float find_threshold(uint32_t *local_widths, int start, int end) {
    uint32_t sorted[9];
    for (int i = 0; i < 9; i++) {
        sorted[i] = local_widths[start + i] & 0x7FFFFFFFU;
    }
    qsort(sorted, 9, sizeof(uint32_t), uint32_cmp);
    float unit = (float)sorted[2];  // Approximate narrow unit as third smallest
    return unit * THRESHOLD_MULTIPLIER;
}

/**
 * @brief Decode character with local threshold
 */
char decode_local(uint32_t *widths) {
    int binary[9];
    float total = 0.0f;
    for (int j = 0; j < 9; j++) {
        uint32_t dt = widths[j] & 0x7FFFFFFFU;
        total += (float)dt;
    }
    float unit = total / 15.0f;
    float th = unit * THRESHOLD_MULTIPLIER;
    for (int j = 0; j < 9; j++) {
        uint32_t dt = widths[j] & 0x7FFFFFFFU;
        binary[j] = (dt > th) ? 1 : 0;
    }
    return decode_char(binary);
}

/**
 * @brief Decode barcode and process result
 */
void decode_and_process(void) {
    uint32_t local_widths[MAX_WIDTHS];
    memcpy(local_widths, (uint32_t*)widths, sizeof(widths));

    // Filter out ultra-short widths (noise)
    int filtered_num = 0;
    for (int i = 0; i < num_widths; i++) {
        uint32_t dt_us = local_widths[i] & 0x7FFFFFFFU;
        if (dt_us >= MIN_VALID_WIDTH_US) {
            local_widths[filtered_num++] = local_widths[i];
        } else {
            printf("Filtered noise width: %u us at index %d\n", dt_us, i);
        }
    }

    // Force trim leading space if present (quiet zone) and longer than threshold
    int start_idx = 0;
    uint32_t leading_dt = local_widths[0] & 0x7FFFFFFFU;
    if (filtered_num > 0 && !(local_widths[0] & 0x80000000U) && leading_dt > LONG_WIDTH_US) {
        start_idx = 1;
        printf("Trimmed leading space (force).\n");
    }

    // Force trim trailing space if present (quiet zone) and longer than threshold
    int end_idx = filtered_num;
    uint32_t trailing_dt = local_widths[end_idx-1] & 0x7FFFFFFFU;
    if (end_idx > start_idx && !(local_widths[end_idx-1] & 0x80000000U) && trailing_dt > LONG_WIDTH_US) {
        end_idx--;
        printf("Trimmed trailing space (force).\n");
    }

    int trimmed_num = end_idx - start_idx;
    if (trimmed_num < 9) {
        printf("After trim, too short: %d widths\n", trimmed_num);
        return;
    }

    // Only decode if exactly 29 widths after trim
    if (trimmed_num != 29) {
        printf("Not 29 widths after trim (%d), skipping decode.\n", trimmed_num);
        num_widths = 0;
        return;
    }

    // Print raw widths for debugging (filtered and trimmed)
    printf("Raw widths (%d filtered/trimmed):\n", trimmed_num);
    for (int i = start_idx; i < end_idx; i++) {
        bool is_bar = (local_widths[i] & 0x80000000U) != 0;
        uint32_t dt_us = local_widths[i] & 0x7FFFFFFFU;
        printf("%s%u ", is_bar ? "B" : "S", dt_us);
    }
    printf("\n");

    printf("\n");

    // Attempt to decode forward with local thresholds per character
    printf("Decoding characters:\n");
    char result[100] = {0};
    int res_idx = 0;
    int i = start_idx;
    while (i + 8 < end_idx) {
        char c = decode_local(&local_widths[i]);
        
        // Print binary for logging
        int binary[9];
        float total = 0.0f;
        for (int j = 0; j < 9; j++) {
            uint32_t dt = local_widths[i + j] & 0x7FFFFFFFU;
            total += (float)dt;
        }
        float unit = total / 15.0f;
        float th = unit * THRESHOLD_MULTIPLIER;
        float unit_k = unit / 1000.0f;
        float th_k = th / 1000.0f;
        float total_k = total / 1000.0f;
        
        printf("Char at pos %d: unit=%.1f k th=%.1f k (total=%.1f k)\n", 
               i - start_idx, unit_k, th_k, total_k);
        printf("Per-element: ");
        for (int j = 0; j < 9; j++) {
            uint32_t dt = local_widths[i + j] & 0x7FFFFFFFU;
            binary[j] = (dt > th) ? 1 : 0;
            bool is_bar = (local_widths[i + j] & 0x80000000U) != 0;
            int elem_pos = (j / 2) + 1;
            char elem_type = is_bar ? 'B' : 'S';
            float width_k = dt / 1000.0f;
            printf("%c%d:%.0f k(%d %s%.0f k) ", elem_type, elem_pos, width_k, 
                   binary[j], binary[j] ? ">" : "<=", th_k);
        }
        printf("\n");
        printf(" -> %c\n", c);
        
        if (c == '?') break;
        result[res_idx++] = c;
        i += 9;
        if (i >= end_idx) break;
        if ((local_widths[i] & 0x80000000U) != 0) break;
        i += 1;
    }

    bool valid = (i == end_idx && res_idx > 1 && result[0] == '*' && result[res_idx - 1] == '*');
    if (valid) {
        result[res_idx - 1] = 0;
        printf("\nDecoded barcode: %s\n", result + 1);
        strcpy(decoded_barcode, result + 1);
        return;
    } else if (i == end_idx && res_idx > 1) {
        result[res_idx] = 0;
        printf("\nDecoded barcode (no start/stop validation): %s\n", result);
        strcpy(decoded_barcode, result);
    }

    // Try reverse
    printf("\nTrying reverse scan...\n");
    int length = end_idx - start_idx;
    for (int j = 0; j < length / 2; j++) {
        uint32_t temp = local_widths[start_idx + j];
        local_widths[start_idx + j] = local_widths[end_idx - 1 - j];
        local_widths[end_idx - 1 - j] = temp;
    }

    if (length > 0 && !(local_widths[start_idx] & 0x80000000U)) {
        start_idx++;
        length--;
        printf("Re-trimmed leading space in reverse.\n");
    }
    if (length > 0 && !(local_widths[start_idx + length - 1] & 0x80000000U)) {
        length--;
        printf("Re-trimmed trailing space in reverse.\n");
    }
    end_idx = start_idx + length;
    trimmed_num = length;
    if (trimmed_num < 9) {
        printf("After reverse trim, too short: %d widths\n", trimmed_num);
        return;
    }

    // Print reversed widths
    printf("Reversed widths (%d):\n", trimmed_num);
    for (int k = start_idx; k < end_idx; k++) {
        bool is_bar = (local_widths[k] & 0x80000000U) != 0;
        uint32_t dt_us = local_widths[k] & 0x7FFFFFFFU;
        printf("%s%u ", is_bar ? "B" : "S", dt_us);
    }
    printf("\n");

    // Decode reversed with local thresholds
    printf("Decoding reversed characters:\n");
    memset(result, 0, sizeof(result));
    res_idx = 0;
    i = start_idx;
    while (i + 8 < end_idx) {
        char c = decode_local(&local_widths[i]);
        
        // Print binary for logging
        int binary[9];
        float total = 0.0f;
        for (int j = 0; j < 9; j++) {
            uint32_t dt = local_widths[i + j] & 0x7FFFFFFFU;
            total += (float)dt;
        }
        float unit = total / 15.0f;
        float th = unit * THRESHOLD_MULTIPLIER;
        float unit_k = unit / 1000.0f;
        float th_k = th / 1000.0f;
        float total_k = total / 1000.0f;
        
        printf("Reversed char at pos %d: unit=%.1f k th=%.1f k (total=%.1f k)\n", 
               i - start_idx, unit_k, th_k, total_k);
        printf("Per-element: ");
        for (int j = 0; j < 9; j++) {
            uint32_t dt = local_widths[i + j] & 0x7FFFFFFFU;
            binary[j] = (dt > th) ? 1 : 0;
            bool is_bar = (local_widths[i + j] & 0x80000000U) != 0;
            int elem_pos = (j / 2) + 1;
            char elem_type = is_bar ? 'B' : 'S';
            float width_k = dt / 1000.0f;
            printf("%c%d:%.0f k(%d %s%.0f k) ", elem_type, elem_pos, width_k, 
                   binary[j], binary[j] ? ">" : "<=", th_k);
        }
        printf("\n");
        printf(" -> %c\n", c);
        
        if (c == '?') break;
        result[res_idx++] = c;
        i += 9;
        if (i >= end_idx) break;
        if ((local_widths[i] & 0x80000000U) != 0) break;
        i += 1;
    }

    valid = (i == end_idx && res_idx > 1 && result[0] == '-' && result[res_idx - 1] == '-');
    if (valid) {
        char data[100];
        strcpy(data, result + 1);
        data[res_idx - 2] = '\0';
        for (int k = 0; k < strlen(data); k++) {
            data[k] = reverse_map(data[k]);
        }
        str_reverse(data);
        printf("\nDecoded barcode (reversed): %s\n", data);
        strcpy(decoded_barcode, data);
    } else if (i == end_idx && res_idx > 1) {
        result[res_idx] = 0;
        printf("\nDecoded barcode (reversed, no start/stop validation): %s\n", result);
        strcpy(decoded_barcode, result);
    } else {
        printf("\nNo valid barcode decoded.\n");
    }
}