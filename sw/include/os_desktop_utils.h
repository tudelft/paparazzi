/**
 * @file os_desktop_utils.h
 * @brief Utilities for dynamically integrating applications into the Linux desktop environment.
 * 
 * @details This component enables auto-generation of XDG desktop rules (creating `.desktop` files)
 * directly from the binary executable at runtime. This allows seamless integration into 
 * GNOME/Wayland launchers without requiring elevated system or `sudo` compilation passes.
 */
#ifndef OS_DESKTOP_UTILS_H
#define OS_DESKTOP_UTILS_H

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QIODevice>
#include <QProcess>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QTextStream>
// Widget/styling headers required by EditorLighteningStyle (defined below).
#include <QAbstractSpinBox>
#include <QColor>
#include <QLineEdit>
#include <QPalette>
#include <QPlainTextEdit>
#include <QProxyStyle>
#include <QTextEdit>
#include <QWidget>

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

/**
 * @class EditorLighteningStyle
 * @brief Lightens aggressively dark text-input widgets imposed by dark system themes.
 *
 * @details Many dark GTK/Qt palettes render @c QLineEdit / @c QTextEdit /
 * @c QPlainTextEdit / @c QAbstractSpinBox backgrounds so dark that their borders
 * and contents fail visibility checks. Rather than fighting Qt's global
 * stylesheet (which tends to strip native OS rendering), this proxy style hooks
 * the per-widget @c polish phase and raises only @c QPalette::Base to a readable
 * @c #3a3a3a on input widgets, leaving every other widget untouched. All members
 * are inline, so the class can be shared by including this header from multiple
 * translation units without violating the One Definition Rule.
 *
 * Usage: @code app.setStyle(new EditorLighteningStyle(app.style())); @endcode
 */
class EditorLighteningStyle : public QProxyStyle {
public:
    // Inherit QProxyStyle's constructors (notably the one taking a base QStyle*).
    using QProxyStyle::QProxyStyle;

    /**
     * @brief Invoked automatically just before each widget is shown.
     * @param widget The widget being prepared for display.
     * @details Forces a lighter @c QPalette::Base on text-input widgets only.
     */
    void polish(QWidget *widget) override {
        // Always run the base class polish first.
        QProxyStyle::polish(widget);

        // Only adjust editable text/number input widgets.
        if (qobject_cast<QLineEdit*>(widget) ||
            qobject_cast<QTextEdit*>(widget) ||
            qobject_cast<QPlainTextEdit*>(widget) ||
            qobject_cast<QAbstractSpinBox*>(widget)) {

            QPalette customPalette = widget->palette();
            customPalette.setColor(QPalette::Base, QColor("#3a3a3a"));
            // Apply only to this specific widget, not globally.
            widget->setPalette(customPalette);
        }
    }
};

#endif // OS_DESKTOP_UTILS_H
