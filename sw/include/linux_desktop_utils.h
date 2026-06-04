/**
 * @file linux_desktop_utils.h
 * @brief Utilities for dynamically integrating applications into the Linux desktop environment.
 * 
 * @details This component enables auto-generation of XDG desktop rules (creating `.desktop` files)
 * directly from the binary executable at runtime. This allows seamless integration into 
 * GNOME/Wayland launchers without requiring elevated system or `sudo` compilation passes.
 */
#ifndef LINUX_DESKTOP_UTILS_H
#define LINUX_DESKTOP_UTILS_H

#include <QCoreApplication>
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QProcess>
#include <QString>
#include <QStringList>

/**
 * @brief Bootstraps a Linux `.desktop` file dynamically so the desktop manager recognizes the app.
 * 
 * @param appDesktopFileName The base file name for the `.desktop` file (without extension).
 * @param displayName The generic visual name displayed to the End User.
 * @param appComment A short tooltip explanation displayed by the desktop launcher.
 * @param iconResourcePath Absolute file path to the native icon resource.
 * @param startupWMClass Represents the `WM_CLASS` X11 property to bind window groups.
 * 
 * @details
 * This algorithm bypasses global system folders (`/usr/share/applications`) avoiding 
 * Permission Denied issues. It accurately delegates to the `QStandardPaths::ApplicationsLocation`, 
 * ensuring compatibility across strict sandbox models (like Flatpaks and AppImages).
 * 
 * @note If the icon resource is missing, the application launcher will fallback to the default generic system icon.
 */
inline void installLinuxDesktopIntegration(const QString& appDesktopFileName, const QString& displayName, const QString& appComment, const QString& iconResourcePath, const QString& startupWMClass) {
#if defined(Q_OS_LINUX)
    // Dynamically install desktop integration files so GNOME/Wayland can pick them up dynamically
    QString exePath = QCoreApplication::arguments().at(0);
    if (!exePath.contains("/")) {
        exePath = QStandardPaths::findExecutable(exePath);
    } else {
        exePath = QDir::cleanPath(QDir().absoluteFilePath(exePath));
    }

    // Substitute standard user home locations dynamically into terminal bindings.
    // Why wrap in bash? 
    // Absolute paths can confuse sandboxed environments if they hardcode local host mount paths. 
    // Passing through standard bash executes allows environment path expansion on run.
    QString userName = qgetenv("USER");
    if (!userName.isEmpty() && exePath.startsWith("/home/" + userName + "/")) {
        // Substitute /home/user/ with ~/ internally wrapped in a bash exec so it's fully portable
        exePath.replace(0, ("/home/" + userName).length(), "~");
        exePath = "bash -c \"exec " + exePath + "\"";
    }

    // Follow also XDG rules for file locations, so that we don't need sudo and it 
    // works in flatpaks/snap/other containerized environments without special permissions
    QString appsLocation = QStandardPaths::writableLocation(QStandardPaths::ApplicationsLocation);
    if (!appsLocation.isEmpty()) {
        QDir().mkpath(appsLocation);
        QString desktopFileName = appDesktopFileName + ".desktop";
        QString desktopFilePath = appsLocation + "/" + desktopFileName;
        QFile dfile(desktopFilePath);
        if (dfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&dfile);
            out << "[Desktop Entry]\n"
                << "Version=1.0\n"
                << "Type=Application\n"
                << "Name=" << displayName << "\n"
                << "Comment=" << appComment << "\n"
                << "Exec=" << exePath << "\n"
                << "Icon=" << appDesktopFileName << "\n"
                << "Terminal=false\n"
                << "Categories=Education;Science;Robotics;\n"
                << "StartupNotify=true\n"
                << "StartupWMClass=" << startupWMClass << "\n";
            dfile.close();
        }

        QString iconDir = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation) + "/icons/hicolor/128x128/apps";
        QDir().mkpath(iconDir);
        QString iconFilePath = iconDir + "/" + appDesktopFileName + ".png";
        if (QFile::exists(iconFilePath)) {
            QFile::remove(iconFilePath);
        }
        
        // If the icon resource exists, copy it globally
        if (QFile::exists(iconResourcePath)) {
            QFile::copy(iconResourcePath, iconFilePath);
        }
        
        // Let the system catch up using Qt's native cross-platform process API, discarding any desktop error messages
        // Running standard Linux DBs sync asynchronously
        QProcess::startDetached("/bin/sh", QStringList() << "-c" << QString("update-desktop-database -q \"%1\" >/dev/null 2>&1").arg(appsLocation));
        QString hicolorDir = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation) + "/icons/hicolor";
        QProcess::startDetached("/bin/sh", QStringList() << "-c" << QString("gtk-update-icon-cache -q -t -f \"%1\" >/dev/null 2>&1").arg(hicolorDir));
    }
#endif
}

#endif // LINUX_DESKTOP_UTILS_H
