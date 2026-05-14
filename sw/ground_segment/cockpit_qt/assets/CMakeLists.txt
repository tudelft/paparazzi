
find_package(ECM 1.0.0 REQUIRED NO_MODULE)
set(CMAKE_MODULE_PATH ${ECM_MODULE_PATH})
include(ECMInstallIcons)
include(KDEInstallDirs)

ecm_install_icons(
    ICONS
        16_equinox_gcs.png
        32_equinox_gcs.png
        64_equinox_gcs.png
        128_equinox_gcs.png
        256_equinox_gcs.png

    THEME
        hicolor

    DESTINATION
        ${KDE_INSTALL_ICONDIR}
)

configure_file(
    org.openuas.equinoxgcs.appdata.xml.in
    org.openuas.equinoxgcs.appdata.xml
)

install(FILES ${CMAKE_BINARY_DIR}/assets/org.openuas.equinoxgcs.appdata.xml
    DESTINATION ${KDE_INSTALL_METAINFODIR}
)

install(FILES pprzgcs.svg
        DESTINATION ${KDE_INSTALL_FULL_ICONDIR}/hicolor/scalable/apps
)

install(FILES "org.openuas.equinoxgcs.desktop"
        DESTINATION ${KDE_INSTALL_APPDIR}
)

