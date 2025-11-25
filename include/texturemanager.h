// texturemanager.h
#ifndef TEXTUREMANAGER_H
#define TEXTUREMANAGER_H

#include "SDL3/SDL_log.h"
#include <string>
#include <unordered_map>

struct SDL_Renderer;
struct SDL_Texture;
struct TTF_Font;

class TextureManager {
public:
  // You must give TextureManager a valid SDL_Renderer* at construction time.
  // It keeps that renderer internally so it can turn surfaces → textures.
  TextureManager(SDL_Renderer* renderer_);

  ~TextureManager();

  // Non‐copyable / non‐movable for simplicity
  TextureManager(const TextureManager&) = delete;
  TextureManager& operator=(const TextureManager&) = delete;

  bool load_texture(const char* id, const char* file_path);
  SDL_Texture* get_texture(const char* id) const;

  // (Optional) Query width/height of a loaded texture.
  bool query_texture(const char* id, float& outW, float& outH) const;

private:
  SDL_Renderer* renderer;
  std::unordered_map<std::string, SDL_Texture*> texture_map;
};

class FontManager {
public:
  FontManager() {};

  ~FontManager();

  // Non‐copyable / non‐movable for simplicity
  FontManager(const FontManager&) = delete;
  FontManager& operator=(const FontManager&) = delete;

  bool load_font(const char* id, const char* file_path, int font_size);
  TTF_Font* get_font(const char* id) const;

private:
  std::unordered_map<std::string, TTF_Font*> font_map;
};

class ResourceProvider {
public:
  virtual TextureManager* get_textureManager() = 0;
  virtual FontManager* get_fontManager() = 0;
  virtual ~ResourceProvider() = default;
};

class GameResources : public ResourceProvider {
public:
  GameResources(SDL_Renderer* renderer_)
      : renderer(renderer_), textureManager(renderer_) {
    load_common_resources();
  }

  TextureManager* get_textureManager() override { return &textureManager; }
  FontManager* get_fontManager() override { return &fontManager; }

  void load_common_resources() {
    if (!textureManager.load_texture("player", "resources/collated_grid.png")) {
      SDL_Log("Failed to load 'player' texture");
    }
    if (!fontManager.load_font("default", "resources/Roboto-Regular.ttf", 16)) {
      SDL_Log("Failed to load 'default' font");
    }
    if (!fontManager.load_font("stopwatch",
                               "resources/DSEG7Classic-Regular.ttf", 32)) {
      SDL_Log("Failed to load 'stopwatch' font");
    }
  }

private:
  SDL_Renderer* renderer;
  TextureManager textureManager;
  FontManager fontManager;
};

#endif
