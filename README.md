# Collie Kart Portable — P001 Systems Check

**Status:** Experimental. This is a UI-only 3GX plugin prototype for Mario Kart 7 (MK7). It is not a learning agent.

## Tests

- Verify that the current process has an MK7 Title ID (USA/EUR/JPN).
- Draw a small bottom-screen status indicator (INACTIVE by default).
- Toggle a display-only READY / INACTIVE flag from the CTRPluginFramework menu.
- Display basic diagnostics (region, toggle count, disabled systems).
- Leave game memory, controller inputs, save data, and SD storage untouched.

**Important:** Title ID verification does **not** detect an active race. READY is only a test flag, not permission for autonomous driving.

## Build

The repository includes a GitHub Actions workflow in `.github/workflows/makefile.yml`. Pull requests targeting `master` trigger a build using `pablomk7/libctrpf:0.8.0`. The Makefile intentionally outputs `CTRPluginFramework-BlankTemplate.3gx`, matching the existing artifact step. Rename this compiled file to `CollieKart.3gx` when installing.

If Actions is disabled on a new fork, enable it in the repository's **Actions** tab. Only install the plugin after a successful build, and do not merge this branch merely to start a test.

## Installation (New 3DS with Luma3DS 3GX loader)

1. Back up your MK7 save and any existing plugins. Do not replace `boot.firm`.
2. Confirm your installed MK7 Title ID (FBI), especially on region-changed consoles.
3. Download and unzip the successful GitHub Actions **Plugin 3GX File** artifact on your iPhone.
4. Place just the compiled `.3gx` in your game's SD plugin folder, for example:
   - USA: `SD:/luma/plugins/0004000000030800/CollieKart.3gx`
   - EUR: `SD:/luma/plugins/0004000000030700/CollieKart.3gx`
   - JPN: `SD:/luma/plugins/0004000000030600/CollieKart.3gx`
5. Do not use `default.3gx`. Do not load CTGP-7 or other MK7 game plugins for the first test.
6. Enable **Plugin Loader** via Luma3DS Rosalina and launch Mario Kart 7.
7. Open the CTRPF menu (usually Select), choose **Show diagnostics**, toggle READY / INACTIVE, and check the small bottom-screen label when the menu is closed.
8. Try MK7's title menu and a solo Time Trial. Normal driving must remain unaffected.

## If the test fails

Disable Plugin Loader in Rosalina, close the game, remove this `.3gx` using the microSD adapter, and try again after reporting the exact issue. No learning data should exist for P001.

The plugin uses a generic bottom-screen location near the lower edge, **not yet verified to be unobtrusive in MK7**. Please report overlap or flicker so we can adjust it.
