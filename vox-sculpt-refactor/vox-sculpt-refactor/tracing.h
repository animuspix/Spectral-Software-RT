#pragma once

namespace tracing
{
    class integration
    {
    public:
        // Allow different "render modes" for different user activities (editing, previews, final output to file)
        // Each render-mode uses a specialized integrator; the "final" modes are heavily pipelined and designed to be one-offs,
        // the TO_FILE mode has no communication with regular window paints + copies output directly to an image buffer, and
        // the EDIT mode renders in discrete, nearly stateless frames (so that it can respond immediately to changes in the
        // volume, camera position, etc)
        enum RENDER_MODES
        {
            RENDER_MODE_EDIT,
            RENDER_MODE_FINAL_PREVIEW,
            RENDER_MODE_FINAL_TO_FILE
            // RENDER_MODE_PAINTING
            // RENDER_MODE_SCULPTING
            // ...etc...
        };

        static void set_render_mode(RENDER_MODES mode);
        static void clear_render_state();
        static void image_integrator();
        static void init();
    };
}