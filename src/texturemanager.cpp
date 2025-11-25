#include "texturemanager.h"
#include "SDL3_ttf/SDL_ttf.h"
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <iostream>
#include <unordered_map>

// You must give TextureManager a valid SDL_Renderer* at construction time.
// It keeps that renderer internally so it can turn surfaces → textures.
TextureManager::TextureManager(SDL_Renderer* renderer) : renderer(renderer) {}

TextureManager::~TextureManager() {
  // Destroy all SDL_Texture* that we have loaded.
  for (auto& kv : texture_map) {
    if (kv.second) {
      SDL_DestroyTexture(kv.second);
    }
  }
}

// Try to load a texture from `filePath` and store it under the key `id`.
// Returns true if successful (or if the key already existed).
bool TextureManager::load_texture(const char* id, const char* file_path) {
  // If we've already loaded “id”, do nothing.
  if (texture_map.count(id)) {
    return true;
  }

  SDL_Surface* surf = IMG_Load(file_path);
  if (!surf) {
    std::cerr << "IMG_Load Error (" << file_path << "): " << SDL_GetError()
              << "\n";
    return false;
  }

  SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
  SDL_DestroySurface(surf);

  if (!tex) {
    std::cerr << "SDL_CreateTextureFromSurface Error (" << file_path
              << "): " << SDL_GetError() << "\n";
    return false;
  }

  texture_map[id] = tex;
  return true;
}

// Retrieve the texture pointer for a given id (returns nullptr if not found).
TTF_Font* FontManager::get_font(const char* id) const {
  auto it = font_map.find(id);
  if (it == font_map.end())
    return nullptr;
  return it->second;
}

// (Optional) Query width/height of a loaded texture.
bool TextureManager::query_texture(const char* id, float& outW,
                                   float& outH) const {
  SDL_Texture* tex = get_texture(id);
  if (!tex)
    return false;
  if (0 != SDL_GetTextureSize(tex, &outW, &outH)) {
    std::cerr << "SDL_QueryTexture Error: " << SDL_GetError() << "\n";
    return false;
  }
  return true;
}

// Retrieve the texture pointer for a given id (returns nullptr if not found).
SDL_Texture* TextureManager::get_texture(const char* id) const {
  auto it = texture_map.find(id);
  if (it == texture_map.end())
    return nullptr;
  return it->second;
}

bool FontManager::load_font(const char* id, const char* file_path,
                            int font_size) {
  if (font_map.count(id)) {
    return true;
  }

  char* font_path = NULL;
  SDL_asprintf(&font_path, "%s/../%s", SDL_GetBasePath(), file_path);
  TTF_Font* font = TTF_OpenFont(font_path, font_size);
  if (!font) {
    SDL_Log("Couldn't load font: %s", SDL_GetError());
    return false;
  }
  SDL_free(font_path);

  font_map[id] = font;
  return true;
}

FontManager::~FontManager() {
  // Destroy all SDL_Texture* that we have loaded.
  for (auto& kv : font_map) {
    if (kv.second) {
      // free(kv.second);
      TTF_CloseFont(kv.second);
    }
  }
}
