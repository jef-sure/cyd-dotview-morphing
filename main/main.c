#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bus/dgx_spi_esp32.h"
#include "dgx_bits.h"
#include "dgx_colors.h"
#include "dgx_draw.h"
#include "dgx_font.h"
#include "driver/gpio.h"
#include "drivers/ili9341.h"
#include "drivers/vscreen.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fonts/ArialRegular12.h"
#include "fonts/IBMCGALight8x16Light8x1616.h"
#include "fonts/TerminusTTFMedium12.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum
{
    CYD_TFT_MOSI      = GPIO_NUM_13,
    CYD_TFT_MISO      = GPIO_NUM_12,
    CYD_TFT_SCLK      = GPIO_NUM_14,
    CYD_TFT_CS        = GPIO_NUM_15,
    CYD_TFT_DC        = GPIO_NUM_2,
    CYD_TFT_RST       = GPIO_NUM_NC,
    CYD_TFT_BACKLIGHT = GPIO_NUM_21,
    CYD_TFT_SPI_HOST  = SPI2_HOST,
    CYD_TFT_SPI_HZ    = 40 * 1000 * 1000
};

static const char *TAG      = "cyd-life-morphing";
static int         neiby[8] = {-1, 0, 0, 1, -1, -1, 1, 1};
static int         neibx[8] = {0, -1, 1, 0, -1, 1, 1, -1};

typedef struct
{
    int     width;
    int     height;
    int     number_of_set_cells;
    uint8_t cells[0];
} CellMatrix;

static inline int min_int(int a, int b)
{
    return a < b ? a : b;
}

static inline int max_int(int a, int b)
{
    return a > b ? a : b;
}

static inline float max_float(float a, float b)
{
    return a > b ? a : b;
}

typedef struct
{
    int16_t x, y;
} Point;

typedef struct
{
    Point   start;
    Point   end;
    uint8_t start_intensity;
    uint8_t end_intensity;
} Segment;

typedef struct
{
    int      hash_count;
    Segment *segments;
} SegmentsInCell;

typedef struct
{
    int16_t        x;
    int16_t        y;
    bool           occupied;
    SegmentsInCell cell;
} SegmentHashSlot;

typedef struct
{
    SegmentHashSlot *slots;
    int              hash_capacity;
    int              hash_count;
    int              radius;
    int              cell_width;
    int              grid_width;
    int              grid_height;
    int              rlut_limit;
    int             *rlut;
    int             *xcell_offset;
    dgx_screen_t    *vscreen;
    uint8_t         *glow_next;
    uint8_t         *glow_prev;
    Point           *static_points;
    int              static_points_count;
    Point           *fading_points;
    int              fading_points_count;
    uint32_t         cp_from;
    uint32_t         cp_to;
    int              font_y_bottom_max;
    int              font_x_lowest;
} MorphingRender;

static uint32_t hash_xy(int x, int y)
{
    uint32_t h = (uint32_t)(int32_t)x * 73856093u ^ (uint32_t)(int32_t)y * 19349663u;
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    return h;
}

static int hash_next_pow2(int hash_capacity)
{
    if (hash_capacity < 8) {
        return 8;
    }
    hash_capacity--;
    hash_capacity |= hash_capacity >> 1;
    hash_capacity |= hash_capacity >> 2;
    hash_capacity |= hash_capacity >> 4;
    hash_capacity |= hash_capacity >> 8;
    hash_capacity |= hash_capacity >> 16;
    return hash_capacity + 1;
}

static bool hash_init(MorphingRender *hash, int hash_capacity)
{
    if (hash == NULL) {
        return false;
    }
    hash_capacity = hash_next_pow2(hash_capacity);
    hash->slots   = (SegmentHashSlot *)calloc((size_t)hash_capacity, sizeof(SegmentHashSlot));
    if (hash->slots == NULL) {
        return false;
    }
    hash->hash_count    = 0;
    hash->hash_capacity = hash_capacity;
    return true;
}

static bool hash_rehash(MorphingRender *hash, int new_capacity)
{
    SegmentHashSlot *old_slots    = hash->slots;
    int              old_capacity = hash->hash_capacity;
    SegmentHashSlot *new_slots    = (SegmentHashSlot *)calloc((size_t)new_capacity, sizeof(SegmentHashSlot));
    if (new_slots == NULL) {
        return false;
    }

    hash->slots         = new_slots;
    hash->hash_capacity = new_capacity;
    hash->hash_count    = 0;

    for (int i = 0; i < old_capacity; i++) {
        if (!old_slots[i].occupied) {
            continue;
        }
        uint32_t mask  = (uint32_t)new_capacity - 1u;
        uint32_t index = hash_xy(old_slots[i].x, old_slots[i].y) & mask;
        while (new_slots[index].occupied) {
            index = (index + 1u) & mask;
        }
        new_slots[index] = old_slots[i];
        hash->hash_count++;
    }

    free(old_slots);
    return true;
}

static SegmentHashSlot *hash_find_slot(const MorphingRender *hash, int x, int y, bool for_insert)
{
    if (hash == NULL || hash->slots == NULL || hash->hash_capacity == 0) {
        return NULL;
    }

    uint32_t         mask        = (uint32_t)hash->hash_capacity - 1u;
    uint32_t         index       = hash_xy(x, y) & mask;
    SegmentHashSlot *first_empty = NULL;

    for (int probed = 0; probed < hash->hash_capacity; probed++) {
        SegmentHashSlot *slot = &hash->slots[index];
        if (!slot->occupied) {
            if (first_empty == NULL) {
                first_empty = slot;
            }
            break;
        }
        if (slot->x == (int16_t)x && slot->y == (int16_t)y) {
            return slot;
        }
        index = (index + 1u) & mask;
    }

    return for_insert ? first_empty : NULL;
}

void hash_add(MorphingRender *hash, int x, int y, Segment segment)
{
    if (hash == NULL) {
        return;
    }
    if (hash->slots == NULL && !hash_init(hash, 16)) {
        return;
    }
    if (hash->hash_count * 4 >= hash->hash_capacity * 3) {
        if (!hash_rehash(hash, hash->hash_capacity * 2)) {
            return;
        }
    }

    SegmentHashSlot *slot = hash_find_slot(hash, x, y, true);
    if (slot == NULL) {
        return;
    }

    if (!slot->occupied) {
        slot->occupied        = true;
        slot->x               = (int16_t)x;
        slot->y               = (int16_t)y;
        slot->cell.hash_count = 0;
        slot->cell.segments   = NULL;
        hash->hash_count++;
    }

    Segment *grown = (Segment *)realloc(slot->cell.segments, (size_t)(slot->cell.hash_count + 1) * sizeof(Segment));
    if (grown == NULL) {
        return;
    }
    slot->cell.segments                        = grown;
    slot->cell.segments[slot->cell.hash_count] = segment;
    slot->cell.hash_count++;
}

void hash_clear(MorphingRender *hash)
{
    if (hash == NULL || hash->slots == NULL) {
        return;
    }
    for (int i = 0; i < hash->hash_capacity; i++) {
        if (hash->slots[i].occupied) {
            free(hash->slots[i].cell.segments);
        }
        hash->slots[i].occupied        = false;
        hash->slots[i].cell.hash_count = 0;
        hash->slots[i].cell.segments   = NULL;
    }
    hash->hash_count = 0;
}

SegmentsInCell hash_get(const MorphingRender *hash, int x, int y)
{
    SegmentsInCell   empty = {.hash_count = 0, .segments = NULL};
    SegmentHashSlot *slot  = hash_find_slot(hash, x, y, false);
    if (slot == NULL) {
        return empty;
    }
    return slot->cell;
}

void hash_free(MorphingRender *hash)
{
    hash_clear(hash);
    if (hash == NULL) {
        return;
    }
    free(hash->slots);
    hash->slots         = NULL;
    hash->hash_capacity = 0;
    hash->hash_count    = 0;
}

#define CELL_OFFSET(x, y, width) ((y) * (width) + (x))

bool is_cell_set(uint8_t *cells, int x, int y, int width)
{
    size_t idx = CELL_OFFSET(x, y, width);
    return (cells[idx / 8u] & (1 << (idx % 8u))) != 0;
}

void mark_cell_set(uint8_t *cells, int x, int y, int width, bool set)
{
    size_t idx = CELL_OFFSET(x, y, width);
    if (set) {
        cells[idx / 8u] |= (1 << (idx % 8u));
    } else {
        cells[idx / 8u] &= ~(1 << (idx % 8u));
    }
}

CellMatrix *create_cell_matrix(dgx_font_t *font, uint32_t codePoint, int font_y_bottom_max, int font_x_lowest)
{
    if (font == NULL || font->xWidest <= 0 || font->yAdvance <= 0) {
        return NULL;
    }

    int16_t        xAdvance;
    const glyph_t *g = dgx_font_find_glyph(codePoint, font, &xAdvance);
    if (g == NULL) {
        return NULL;
    }

    int         width      = font->xWidest - (font_x_lowest < 0 ? font_x_lowest : 0);
    int         height     = font_y_bottom_max - font->yOffsetLowest;
    int         x_shift    = font_x_lowest < 0 ? -font_x_lowest : 0;
    size_t      cell_bytes = ((size_t)width * (size_t)height + 7u) / 8u;
    CellMatrix *matrix     = (CellMatrix *)calloc(1, sizeof(*matrix) + cell_bytes);
    if (matrix == NULL) {
        return NULL;
    }

    if (font->f_type == DGX_FONT_DOTS) {
        for (int di = 0; di < g->number_of_dots; di++) {
            int x = g->dots[di].x + g->xOffset + x_shift;
            int y = g->dots[di].y + g->yOffset - font->yOffsetLowest;
            if (x >= 0 && x < width && y >= 0 && y < height && !is_cell_set(matrix->cells, x, y, width)) {
                mark_cell_set(matrix->cells, x, y, width, true);
                matrix->number_of_set_cells++;
            }
        }
    } else {
        dgx_bw_bitmap_t bmap = dgx_bw_bitmap_make_of((uint8_t *)g->bitmap, g->width, g->height, font->f_type == DGX_FONT_BITMAP_STREAM);
        for (int by = 0; by < g->height; ++by) {
            for (int bx = 0; bx < g->width; bx++) {
                bool pix = dgx_bw_bitmap_get_pixel(&bmap, bx, by);
                if (pix) {
                    int x = bx + g->xOffset + x_shift;
                    int y = by + g->yOffset - font->yOffsetLowest;
                    if (x >= 0 && x < width && y >= 0 && y < height && !is_cell_set(matrix->cells, x, y, width)) {
                        mark_cell_set(matrix->cells, x, y, width, true);
                        matrix->number_of_set_cells++;
                    }
                }
            }
        }
    }
    matrix->width  = width;
    matrix->height = height;
    return matrix;
}

static inline float smoothstep3(float t);

static void morphing_render_free(MorphingRender *render)
{
    if (render == NULL) {
        return;
    }
    dgx_screen_destroy(&render->vscreen);
    free(render->rlut);
    free(render->xcell_offset);
    free(render->glow_next);
    free(render->glow_prev);
    free(render->static_points);
    free(render->fading_points);
    hash_free(render);
}

static void morphing_render_clear_transition(MorphingRender *render)
{
    if (render == NULL) {
        return;
    }
    hash_clear(render);
    free(render->static_points);
    free(render->fading_points);
    render->static_points       = NULL;
    render->fading_points       = NULL;
    render->static_points_count = 0;
    render->fading_points_count = 0;
}

static bool morphing_render_init(MorphingRender *render, int width, int height, int cell_width)
{
    if (render == NULL || width <= 0 || height <= 0 || cell_width <= 0) {
        return false;
    }
    int font_y_bottom_max = render->font_y_bottom_max;
    int font_x_lowest     = render->font_x_lowest;
    memset(render, 0, sizeof(*render));
    render->font_y_bottom_max = font_y_bottom_max;
    render->font_x_lowest     = font_x_lowest;
    if (!hash_init(render, 16)) {
        return false;
    }

    render->cell_width   = cell_width;
    render->grid_width   = width * cell_width;
    render->grid_height  = height * cell_width;
    render->radius       = cell_width / 2 + cell_width / 4;
    render->rlut_limit   = render->radius * render->radius;
    render->vscreen      = dgx_vscreen_init(render->grid_width, render->grid_height, 16, DgxScreenRGB);
    render->rlut         = (int *)malloc((size_t)render->rlut_limit * sizeof(*render->rlut));
    render->xcell_offset = (int *)malloc((size_t)(render->radius + 1) * sizeof(*render->xcell_offset));
    size_t pixel_count   = (size_t)render->grid_width * render->grid_height;
    render->glow_next    = (uint8_t *)calloc(pixel_count, sizeof(*render->glow_next));
    render->glow_prev    = (uint8_t *)calloc(pixel_count, sizeof(*render->glow_prev));
    if (render->vscreen == NULL || render->rlut == NULL || render->xcell_offset == NULL || render->glow_next == NULL || render->glow_prev == NULL) {
        morphing_render_free(render);
        return false;
    }

    for (int i = 0; i < render->rlut_limit; i++) {
        float lut_t     = (float)i / (float)render->rlut_limit;
        render->rlut[i] = (int)(255.0f * (1.0f - smoothstep3(lut_t)));
    }
    for (int dy = 0; dy <= render->radius; dy++) {
        int remaining            = render->rlut_limit - 1 - dy * dy;
        render->xcell_offset[dy] = remaining >= 0 ? (int)sqrtf((float)remaining) : -1;
    }
    return true;
}

static bool create_cell_morphing(MorphingRender *render, dgx_font_t *font, dgx_screen_t *screen, uint32_t cFrom, uint32_t cTo)
{
    if (render == NULL || font == NULL) {
        return false;
    }
    CellMatrix *from = create_cell_matrix(font, cFrom, render->font_y_bottom_max, render->font_x_lowest);
    CellMatrix *to   = create_cell_matrix(font, cTo, render->font_y_bottom_max, render->font_x_lowest);
    if (from == NULL && to == NULL) {
        ESP_LOGE(TAG, "create_cell_morphing: no glyph found for '%c' or '%c'", (char)cFrom, (char)cTo);
        free(from);
        free(to);
        return false;
    }
    int width      = from ? from->width : to->width;
    int height     = from ? from->height : to->height;
    int cell_width = min_int(screen->width / width, screen->height / height);
    if (render->vscreen == NULL && !morphing_render_init(render, width, height, cell_width)) {
        ESP_LOGE(TAG, "create_cell_morphing: morphing_render_init failed (w=%d h=%d cw=%d)", width, height, cell_width);
        free(from);
        free(to);
        return false;
    }
    if (render->vscreen == NULL || render->grid_width != width * cell_width || render->grid_height != height * cell_width) {
        ESP_LOGE(TAG, "create_cell_morphing: grid mismatch grid=%dx%d expected=%dx%d (w=%d h=%d cw=%d)", render->grid_width, render->grid_height,
                 width * cell_width, height * cell_width, width, height, cell_width);
        free(from);
        free(to);
        return false;
    }
    morphing_render_clear_transition(render);
    if ((!from || from->number_of_set_cells == 0) && (!to || to->number_of_set_cells == 0)) {
        ESP_LOGE(TAG, "create_cell_morphing: both glyphs '%c' and '%c' have no set cells", (char)cFrom, (char)cTo);
        free(from);
        free(to);
        return false;
    }
    Point center_point = {.x = (width / 2) * cell_width + (cell_width / 2), .y = (height / 2) * cell_width + (cell_width / 2)};
    if (!from || from->number_of_set_cells == 0) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (is_cell_set(to->cells, x, y, to->width)) {
                    Segment s = (Segment){
                        .end             = {.x = x * cell_width + (cell_width / 2), .y = y * cell_width + (cell_width / 2)},
                        .start           = center_point,
                        .start_intensity = 0,
                        .end_intensity   = 255
                    };
                    hash_add(render, x, y, s);
                }
            }
        }
        render->static_points_count = 0;
        free(from);
        free(to);
        return true;
    }
    if (!to || to->number_of_set_cells == 0) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (is_cell_set(from->cells, x, y, width)) {
                    Segment s = (Segment){
                        .start           = {.x = x * cell_width + (cell_width / 2), .y = y * cell_width + (cell_width / 2)},
                        .end             = center_point,
                        .start_intensity = 255,
                        .end_intensity   = 0
                    };
                    hash_add(render, x, y, s);
                }
            }
        }
        render->static_points_count = 0;
        free(from);
        free(to);
        return true;
    }
    uint8_t *source_used = (uint8_t *)calloc((size_t)(width * height + 7) / 8, sizeof(uint8_t));
    if (source_used == NULL) {
        ESP_LOGE(TAG, "create_cell_morphing: calloc source_used failed");
        free(from);
        free(to);
        return false;
    }
    Point *static_points = (Point *)calloc(to->number_of_set_cells, sizeof(Point));
    if (static_points == NULL) {
        ESP_LOGE(TAG, "create_cell_morphing: calloc static_points failed");
        free(from);
        free(to);
        free(source_used);
        return false;
    }
    Point *fading_points = (Point *)calloc(from->number_of_set_cells, sizeof(Point));
    if (fading_points == NULL) {
        ESP_LOGE(TAG, "create_cell_morphing: calloc fading_points failed");
        free(from);
        free(to);
        free(source_used);
        free(static_points);
        return false;
    }
    uint8_t *dest_second_phase = (uint8_t *)calloc((size_t)(width * height + 7) / 8, sizeof(uint8_t));
    if (dest_second_phase == NULL) {
        ESP_LOGE(TAG, "create_cell_morphing: calloc dest_second_phase failed");
        free(from);
        free(to);
        free(source_used);
        free(static_points);
        free(fading_points);
        return false;
    }
    int fading_points_count       = 0;
    int static_points_count       = 0;
    render->static_points         = static_points;
    render->fading_points         = fading_points;
    bool is_second_phase_required = false;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            bool    from_cell = is_cell_set(from->cells, x, y, width);
            bool    to_cell   = is_cell_set(to->cells, x, y, width);
            uint8_t cell_case = from_cell + to_cell * 2;
            if (cell_case == 0) continue; // Both cells are off
            if (cell_case == 3) {
                static_points[static_points_count++] = (Point){.x = x * cell_width + (cell_width / 2), .y = y * cell_width + (cell_width / 2)};
            }
            if (cell_case == 2) {
                bool found_source = false;
                for (int i = 0; i < 8; i++) {
                    int nx = x + neibx[i];
                    int ny = y + neiby[i];
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height && is_cell_set(from->cells, nx, ny, width) &&
                        !is_cell_set(source_used, nx, ny, width)) {
                        Segment s = (Segment){
                            .end             = {.x = x * cell_width + (cell_width / 2),  .y = y * cell_width + (cell_width / 2) },
                            .start           = {.x = nx * cell_width + (cell_width / 2), .y = ny * cell_width + (cell_width / 2)},
                            .start_intensity = 255,
                            .end_intensity   = 255
                        };
                        hash_add(render, x, y, s);
                        mark_cell_set(source_used, nx, ny, width, true);
                        found_source = true;
                        break;
                    }
                }
                if (!found_source) {
                    is_second_phase_required = true;
                    mark_cell_set(dest_second_phase, x, y, width, true);
                }
            }
        }
    }
    if (is_second_phase_required) {
        const int max_scale = max_int(from->width, from->height);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (!is_cell_set(dest_second_phase, x, y, width)) continue;
                bool found_source = false;
                for (int scale = 2; scale <= max_scale && !found_source; ++scale) {
                    // d — смещение вдоль стороны квадрата от осевой точки к углу
                    for (int d = 0; d <= scale && !found_source; ++d) {
                        for (int k = 0; k < 4 && !found_source; ++k) {
                            // при d == scale это углы; их целиком покрывают верхняя и нижняя стороны
                            if (d == scale && neibx[k] != 0) continue;
                            // вектор вдоль стороны, перпендикулярный оси: (-ay, ax)
                            const int px = -neiby[k];
                            const int py = neibx[k];
                            for (int sgn = -1; sgn <= 1; sgn += 2) {
                                if (d == 0 && sgn == 1) continue; // при d == 0 обе точки совпадают
                                int nx = x + neibx[k] * scale + sgn * d * px;
                                int ny = y + neiby[k] * scale + sgn * d * py;
                                if (nx >= 0 && nx < from->width && ny >= 0 && ny < from->height && is_cell_set(from->cells, nx, ny, from->width) &&
                                    !is_cell_set(source_used, nx, ny, from->width)) {
                                    Segment s = (Segment){
                                        .end             = {.x = x * cell_width + (cell_width / 2),  .y = y * cell_width + (cell_width / 2) },
                                        .start           = {.x = nx * cell_width + (cell_width / 2), .y = ny * cell_width + (cell_width / 2)},
                                        .start_intensity = 255,
                                        .end_intensity   = 255
                                    };
                                    hash_add(render, x, y, s);
                                    mark_cell_set(source_used, nx, ny, from->width, true);
                                    found_source = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (!found_source) {
                    Segment s = (Segment){
                        .end             = {.x = x * cell_width + (cell_width / 2), .y = y * cell_width + (cell_width / 2)},
                        .start           = center_point,
                        .start_intensity = 0,
                        .end_intensity   = 255
                    };
                    hash_add(render, x, y, s);
                }
            }
        }
    }
    render->static_points_count = static_points_count;
    for (int y = 0; y < from->height; y++) {
        for (int x = 0; x < from->width; x++) {
            if (is_cell_set(from->cells, x, y, from->width) && !is_cell_set(to->cells, x, y, from->width) &&
                !is_cell_set(source_used, x, y, from->width)) {
                fading_points[fading_points_count++] = (Point){.x = x * cell_width + (cell_width / 2), .y = y * cell_width + (cell_width / 2)};
            }
        }
    }
    render->fading_points_count = fading_points_count;
    free(dest_second_phase);
    free(source_used);
    free(from);
    free(to);
    return true;
}

static dgx_screen_t *cyd_init_display(void)
{
    dgx_bus_protocols_t *bus =
        dgx_spi_init(CYD_TFT_SPI_HOST, SPI_DMA_CH_AUTO, CYD_TFT_MOSI, CYD_TFT_MISO, CYD_TFT_SCLK, CYD_TFT_CS, CYD_TFT_DC, CYD_TFT_SPI_HZ, 0);
    if (bus == NULL) {
        ESP_LOGE(TAG, "DGX SPI init failed");
        return NULL;
    }

    dgx_screen_t *screen = dgx_ili9341_init(bus, CYD_TFT_RST, CYD_TFT_BACKLIGHT, 16, DgxScreenRGB);
    if (screen == NULL) {
        ESP_LOGE(TAG, "ILI9341 init failed");
        return NULL;
    }

    dgx_ili9341_orientation(screen, DgxScreenRightLeft, DgxScreenTopBottom, true);
    dgx_fill_rectangle(screen, 0, 0, screen->width, screen->height, DGX_BLACK(DGX_RGB_16));
    return screen;
}

static inline float smoothstep3(float t)
{
    return t * t * (3 - 2 * t);
}

Point inner_point(float t, Point start, Point end)
{
    return (Point){.x = start.x + (int)((end.x - start.x) * t), .y = start.y + (int)((end.y - start.y) * t)};
}

static inline uint32_t div255(uint32_t n)
{
    return (n + 1 + (n >> 8)) >> 8;
}

static void collect_glow(MorphingRender *transformation, uint8_t *glow, Point point, uint8_t intensity)
{
    if (intensity == 0) {
        return;
    }

    int y0 = max_int(point.y - transformation->radius, 0);
    int y1 = min_int(point.y + transformation->radius, transformation->grid_height - 1);

    for (int y = y0; y <= y1; y++) {
        int dy     = y - point.y;
        int max_dx = transformation->xcell_offset[abs(dy)];
        if (max_dx < 0) {
            continue;
        }

        int x0 = max_int(point.x - max_dx, 0);
        int x1 = min_int(point.x + max_dx, transformation->grid_width - 1);
        if (x0 > x1) {
            continue;
        }

        int dy2        = dy * dy;
        int row_offset = CELL_OFFSET(0, y, transformation->grid_width);
        for (int x = x0; x <= x1; x++) {
            int dx           = x - point.x;
            int distance     = dx * dx + dy2;
            int falloff      = transformation->rlut[distance];
            int contribution = div255(falloff * intensity);
            int idx          = row_offset + x;
            int collected    = glow[idx] + contribution;
            glow[idx]        = min_int(collected, 255);
        }
    }
}

void collect_initial_glow(MorphingRender *transformation)
{
    for (int i = 0; i < transformation->static_points_count; i++) {
        collect_glow(                         //
            transformation,                   //
            transformation->glow_next,        //
            transformation->static_points[i], //
            255                               //
        );
    }
}

void render_morphing(float t, MorphingRender *transformation)
{
    if (transformation == NULL || transformation->vscreen == NULL) {
        return;
    }

    uint8_t *glow_accu        = transformation->glow_prev;
    transformation->glow_prev = transformation->glow_next;
    transformation->glow_next = glow_accu;
    memset(transformation->glow_next, 0, (size_t)transformation->grid_width * transformation->grid_height);

    float t_tail = max_float(t * 1.5f - 0.5f, 0.0f);
    for (int i = 0; i < transformation->hash_capacity; i++) {
        SegmentHashSlot *slot = &transformation->slots[i];
        if (!slot->occupied) {
            continue;
        }
        for (int j = 0; j < slot->cell.hash_count; j++) {
            Segment *segment   = &slot->cell.segments[j];
            Point    tail      = inner_point(t_tail, segment->start, segment->end);
            Point    head      = inner_point(t, segment->start, segment->end);
            uint8_t  intensity = segment->start_intensity == segment->end_intensity
                                   ? segment->end_intensity
                                   : segment->start_intensity + ((segment->end_intensity - segment->start_intensity) * t);
            collect_glow(transformation, transformation->glow_next, tail, intensity / 2);
            collect_glow(transformation, transformation->glow_next, head, intensity / 2);
        }
    }

    for (int i = 0; i < transformation->static_points_count; i++) {
        collect_glow(transformation, transformation->glow_next, transformation->static_points[i], 255);
    }

    uint8_t fade_intensity = (uint8_t)(255.0f * (1.0f - t));
    for (int i = 0; i < transformation->fading_points_count; i++) {
        collect_glow(transformation, transformation->glow_next, transformation->fading_points[i], fade_intensity);
    }

    uint32_t  blend       = (uint32_t)(256.0f * smoothstep3(t));
    uint16_t *pixels      = (uint16_t *)((dgx_vscreen_t *)transformation->vscreen)->v_array;
    int       pixel_count = transformation->grid_width * transformation->grid_height;
    for (int i = 0; i < pixel_count; i++) {
        uint8_t  intensity           = (uint8_t)((transformation->glow_next[i] * blend + transformation->glow_prev[i] * (256 - blend)) >> 8);
        uint16_t rgb                 = DGX_RGB_16(intensity, intensity, intensity);
        pixels[i]                    = (uint16_t)((rgb >> 8) | (rgb << 8));
        transformation->glow_next[i] = intensity;
    }
}

typedef enum
{
    ButtonReleased,
    ButtonPressed,
} ButtonState;

void app_main(void)
{
    dgx_screen_t *screen = cyd_init_display();
    if (screen == NULL) {
        return;
    }
    int         font_y_bottom_max = INT32_MIN;
    int         font_x_lowest     = INT32_MAX;
    dgx_font_t *font              = TerminusTTFMedium12();
    for (const glyph_array_t *r = font->glyph_ranges; r->number; ++r) {
        for (int i = 0; i < r->number; i++) {
            const glyph_t *g = r->glyphs + i;
            if (g->yOffset + g->height > font_y_bottom_max) {
                font_y_bottom_max = g->yOffset + g->height;
            }
            if (g->xOffset < font_x_lowest) {
                font_x_lowest = g->xOffset;
            }
        }
    }
    ESP_LOGI(TAG, "Font Y Bottom Max: %d", font_y_bottom_max);
    ESP_LOGI(TAG, "Font X Lowest: %d", font_x_lowest);
    MorphingRender *render = (MorphingRender *)calloc(1, sizeof(*render));
    if (render == NULL) {
        return;
    }
    render->font_y_bottom_max   = font_y_bottom_max;
    render->font_x_lowest       = font_x_lowest;
    bool initial_glow_collected = false;
    //
    static const char *cPoints = "0123456789 AÄBCDEFGHIJKLMNOÖPQRSTUÜVWXYZaäbcdefghijklmnoöpqrsßtuüvwxyz "
                                 "АБВГДЕЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдеёжзийклмнопрстуфхцчшщъыьэюя";
    //
    size_t idx = 0;
    while (true) {
        uint32_t codepointFrom;
        codepointFrom = decodeUTF8next(cPoints, &idx);
        if (cPoints[idx] == '\0') {
            idx = 0;
        }
        uint32_t codepointTo;
        size_t   idx_backup = idx;
        codepointTo         = decodeUTF8next(cPoints, &idx);
        idx                 = idx_backup;
        if (!create_cell_morphing(render, font, screen, codepointFrom, codepointTo)) {
            ESP_LOGE(TAG, "Unable to create morph U+%04" PRIX32 " -> U+%04" PRIX32, codepointFrom, codepointTo);
            continue;
        }
        if (!initial_glow_collected) {
            collect_initial_glow(render);
            initial_glow_collected = true;
        }

        int64_t  fps_started = esp_timer_get_time();
        uint32_t frame_count = 0;
        int64_t  start_time  = esp_timer_get_time();
        while (true) {
            float t = (float)(esp_timer_get_time() - start_time) / 1000000.0f;
            if (t > 1.0f) {
                t = 1.0f;
            }
            render_morphing(t, render);
            dgx_vscreen_to_screen(screen, (screen->width - render->grid_width) / 2, (screen->height - render->grid_height) / 2, render->vscreen);
            frame_count++;
            int64_t fps_elapsed = esp_timer_get_time() - fps_started;
            if (fps_elapsed >= 1000000) {
                ESP_LOGI(TAG, "FPS: %.1f", frame_count * 1000000.0 / fps_elapsed);
                fps_started = esp_timer_get_time();
                frame_count = 0;
            }
            if (t >= 1.0f) {
                break;
            }
            vTaskDelay(1);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    morphing_render_free(render);
    free(render);
}