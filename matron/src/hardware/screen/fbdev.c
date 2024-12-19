#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <cairo.h>
#include <SDL2/SDL.h>

#include "hardware/io.h"
#include "hardware/screen/screens.h"

typedef struct _screen_fbdev_priv {
    SDL_Window *window;
    SDL_Surface *window_surface;
    SDL_Surface *draw_surface;
} screen_fbdev_priv_t;

static int screen_fbdev_config(matron_io_t *io, lua_State *l);
static int screen_fbdev_setup(matron_io_t *io);
static void screen_fbdev_destroy(matron_io_t *io);
static void screen_fbdev_paint(matron_fb_t *fb);
static void screen_fbdev_bind(matron_fb_t *fb, cairo_surface_t *surface);

static void screen_fbdev_surface_destroy(void *priv);
static cairo_surface_t *screen_fbdev_surface_create(screen_fbdev_priv_t *priv);

static SDL_Rect windowSize = {
    .x = 0,
    .y = 0,
    .w = 512,
    .h = 256
};

screen_ops_t screen_fbdev_ops = {
    .io_ops.name      = "screen:fbdev",
    .io_ops.type      = IO_SCREEN,
    .io_ops.data_size = sizeof(screen_fbdev_priv_t),
    .io_ops.config    = screen_fbdev_config,
    .io_ops.setup     = screen_fbdev_setup,
    .io_ops.destroy   = screen_fbdev_destroy,

    .paint = screen_fbdev_paint,
    .bind  = screen_fbdev_bind,
};

int screen_fbdev_config(matron_io_t *io, lua_State *l) {
    (void)io;
    (void)l;
    return 0;
}

int screen_fbdev_setup(matron_io_t *io) {
    matron_fb_t *fb = (matron_fb_t *)io;
    fb->surface = screen_fbdev_surface_create((screen_fbdev_priv_t *)io->data);
    if (!fb->surface) {
        fprintf(stderr, "ERROR (%s) failed to create surface\n", io->ops->name);
        return -1;
    }
    fb->cairo = cairo_create(fb->surface);
    return 0;
}

static void screen_fbdev_destroy(matron_io_t *io) {
    matron_fb_t *fb = (matron_fb_t *)io;
    cairo_destroy(fb->cairo);
    cairo_surface_destroy(fb->surface);
}

static void screen_fbdev_paint(matron_fb_t *fb) {
    screen_fbdev_priv_t *priv = fb->io.data;
    cairo_paint(fb->cairo);    
    SDL_BlitScaled(priv->draw_surface, NULL, priv->window_surface, &windowSize);
    SDL_UpdateWindowSurface(priv->window);
}

static void screen_fbdev_bind(matron_fb_t *fb, cairo_surface_t *surface) {
    cairo_set_operator(fb->cairo, CAIRO_OPERATOR_SOURCE);
    cairo_set_source_surface(fb->cairo, surface, 0, 0);
}

static void screen_fbdev_surface_destroy(void *data) {
    screen_fbdev_priv_t *priv = data;

    if (priv == NULL) {
        return;
    }
    SDL_FreeSurface(priv->draw_surface);
    SDL_DestroyWindow(priv->window);
    free(priv);
}

static cairo_surface_t *screen_fbdev_surface_create(screen_fbdev_priv_t *priv) {
    cairo_surface_t *surface;

    SDL_Init(SDL_INIT_VIDEO);
    priv->window = SDL_CreateWindow("matron",
                                    SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,                                 windowSize.w, windowSize.h,
                                    SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    priv->window_surface = SDL_GetWindowSurface(priv->window);
    priv->draw_surface = SDL_CreateRGBSurface(0,
                                              128, 64,
                                              16, 0xf800, 0x000007e0, 0x0000001f,
                                              0);
    surface = cairo_image_surface_create_for_data((unsigned char *)priv->draw_surface->pixels,
                                                  CAIRO_FORMAT_RGB16_565, priv->draw_surface->w, priv->draw_surface->h,
                                                  cairo_format_stride_for_width(CAIRO_FORMAT_RGB16_565, priv->draw_surface->w));
    cairo_surface_set_user_data(surface, NULL, priv, &screen_fbdev_surface_destroy);
    cairo_surface_set_device_scale(surface, MAGNIFY_WIDTH, MAGNIFY_HEIGHT);

    return surface;
}
