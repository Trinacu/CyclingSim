#include <SDL3/SDL.h>
#include <iostream>

void convert_color(SDL_Surface* surf) {
    SDL_LockSurface(surf);

    int w = surf->w;
    int h = surf->h;
    int pitch = surf->pitch;
    auto details = SDL_GetPixelFormatDetails(surf->format);
    int bpp = details->bytes_per_pixel;

    for (int y = 0; y < h; y++) {
        Uint8* row = (Uint8*)surf->pixels + y * pitch;
        for (int x = 0; x < w; x++) {
            Uint8* pixelPtr = row + x * bpp;
            Uint32 rawpixel = 0;

            // 1) Read exactly bpp bytes into a Uint32:
            switch (bpp) {
            case 1:
                rawpixel = *pixelPtr;
                break;
            case 2:
                rawpixel = *(Uint16*)pixelPtr;
                break;
            case 3:
                if (SDL_BYTEORDER == SDL_BIG_ENDIAN) {
                    rawpixel = (pixelPtr[0] << 16) | (pixelPtr[1] << 8) | pixelPtr[2];
                } else {
                    rawpixel = pixelPtr[0] | (pixelPtr[1] << 8) | (pixelPtr[2] << 16);
                }
                break;
            case 4:
                rawpixel = *(Uint32*)pixelPtr;
                break;
            }

            // 2) Extract R,G,B,A:
            Uint8 r, g, b, a;
            SDL_GetRGBA(rawpixel, details, nullptr, &r, &g, &b, &a);

            // 3) Decide if we want to repaint it:
            if (r > 120 && g < 20 && b > 120) {
                Uint8 newR = 255;
                Uint8 newG = 0;
                Uint8 newB = 255;
                // keep same alpha
                Uint32 new_pixel = SDL_MapRGBA(details, nullptr, newR, newG, newB, a);

                // 4) Write exactly bpp bytes back:
                switch (bpp) {
                case 1:
                    *pixelPtr = (Uint8)(new_pixel & 0xFF);
                    break;
                case 2:
                    *(Uint16*)pixelPtr = (Uint16)(new_pixel & 0xFFFF);
                    break;
                case 3:
                    if (SDL_BYTEORDER == SDL_BIG_ENDIAN) {
                        pixelPtr[0] = (new_pixel >> 16) & 0xFF;
                        pixelPtr[1] = (new_pixel >> 8) & 0xFF;
                        pixelPtr[2] = (new_pixel >> 0) & 0xFF;
                    } else {
                        pixelPtr[0] = (new_pixel >> 0) & 0xFF;
                        pixelPtr[1] = (new_pixel >> 8) & 0xFF;
                        pixelPtr[2] = (new_pixel >> 16) & 0xFF;
                    }
                    break;
                case 4:
                    *(Uint32*)pixelPtr = new_pixel;
                    break;
                }
            }
        }
    }

    SDL_UnlockSurface(surf);
}

SDL_Surface* load_transparent_bmp(const char* filename) {
    // The final texture
    SDL_Texture* newTexture = NULL;

    // Load image at specified path
    SDL_Surface* surf = SDL_LoadBMP(filename);

    convert_color(surf);
    // Color key image
    const SDL_PixelFormatDetails* fmt_details = SDL_GetPixelFormatDetails(surf->format);
    SDL_SetSurfaceColorKey(surf, true, SDL_MapRGB(fmt_details, nullptr, 0xFF, 0, 0xFF));

    return surf;
}

const SDL_Texture* load_from_file(SDL_Renderer* renderer, const char* filename) {
    SDL_Surface* surf = load_transparent_bmp(filename);
    if (!surf) {
        std::cerr << "IMG_Load Error: " << SDL_GetError() << "\n";
        return NULL;
    }
    // 2) Create texture
    SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
    if (!tex) {
        std::cerr << "SDL_CreateTextureFromSurface Error: " << SDL_GetError() << "\n";
        SDL_DestroySurface(surf);
        return NULL;
    }

    // 3) Save dimensions
    // int width = surf->w;
    // int height = surf->h;

    SDL_DestroySurface(surf);
    return tex;
}
