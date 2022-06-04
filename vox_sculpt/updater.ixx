export module updater;

import platform;
import geometry;
import camera;

export namespace render_updates
{
	void process()
	{
		// Definitions
		float xrot = 0.0f, yrot = 0.0f; // X, Y rotation (euler angles, we don't support roll atm)
		float z = 0.0f; // Zoom; delta locked at 0.1f (arbitrary)
		const float spinSpd = 360.0f / 30.0f; // Delta-angle locked to 12 degrees (360 degrees in a second @ 30fps)
		const float zSpd = 0.0001f;

		// Volume updates
		xrot = platform::osTestKey(platform::VOX_SCULPT_KEYS::KEY_UP_ARROW) ? spinSpd :
			platform::osTestKey(platform::VOX_SCULPT_KEYS::KEY_DOWN_ARROW) ? -spinSpd : 0.0f;

		yrot = platform::osTestKey(platform::VOX_SCULPT_KEYS::KEY_LEFT_ARROW) ? spinSpd :
			platform::osTestKey(platform::VOX_SCULPT_KEYS::KEY_RIGHT_ARROW) ? -spinSpd : 0.0f;

		z = platform::osTestKey(platform::VOX_SCULPT_KEYS::KEY_RSHIFT) ? zSpd :
			platform::osTestKey(platform::VOX_SCULPT_KEYS::KEY_RCTRL) ? -zSpd : 0.0f;

		// Convert zoom/orbit to object transforms (scale & rotation), then send them over to our volume
		const bool spinning = xrot != 0.0f || yrot != 0.0f;
		const bool zooming = z != 0.0f;
		bool view_resampling = false;
		if (zooming || spinning)
		{
			if (spinning) geometry::spin(xrot, yrot);
			if (zooming) geometry::zoom(z);
			view_resampling = true;
		}
	}
}