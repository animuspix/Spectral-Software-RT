export module spectra;
import vmath;

export namespace spectra
{
    // Super basic sky model - sharp blue fading to orange as y approaches zero
    float sky(float rho, float y)
    {
        float blue = 1.0f - vmath::fabs(0.2f - rho);
        blue *= blue;
        float orange = 1.0f - vmath::fabs(0.7f - rho);
        return vmath::lerp(blue, orange, vmath::fabs(y));
    }

    // Custom film/sensor response curve, kinda follows the references in Zucconi's diffraction tutorial but heavily iterated to give a more film-like
    // spectrum; Zucconi's tutorial is here
    // https://www.alanzucconi.com/2017/07/15/improving-the-rainbow/,
    // aand my Desmos workspace for the curves themselves is here:
    // https://www.desmos.com/calculator/zm9jwr0ngk
    vmath::vec<3> film(float rho)
    {
        return vmath::vec<3>(vmath::fmax(vmath::quadratic(rho, 4.0f, 0.6f, 0.2f, true), 0.0f) +
                             vmath::fmax(vmath::quadratic(rho, 4.0f, 3.0f, 1.0f, true), 0.0f),
                             vmath::fmax(vmath::gaussian(rho, 1.0f, 0.5f, 0.2f, 0.05f), 0.0f),
                             vmath::fmax(vmath::gaussian(rho, 1.0f, 0.0f, 0.55f, 0.2f) *
                                         vmath::quadratic(rho / 0.4f, 1.0f, -0.6f / 0.4f, -2.3f, false) *
                                         vmath::quadratic(rho, 1.0f, 0.95f, 0.0f, false) + 0.1f, 0.0f));
    }

    // Generic diffuse surface colors until I get imgui working
    const float placeholder_spd(vmath::vec<4> coords_and_rho) // [xyz], w (wavelength)
    {
        return vmath::gaussian(coords_and_rho.w(), 2.7f, 0.8f, 0.2f, 1.7f); // Debug red :p
    }
};
