struct SDL_Surface;
struct SDL_Texture;
struct SDL_Renderer;

void convert_color(SDL_Surface* surf);

SDL_Surface* load_transparent_bmp(const char* filename);
SDL_Texture* load_from_file(SDL_Renderer* renderer, const char* filename);
