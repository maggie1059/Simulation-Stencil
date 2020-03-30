#include <QApplication>
#include <QCommandLineParser>
#include "mainwindow.h"
#include <cstdlib>
#include <ctime>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
//    QCommandLineParser parser;
//    parser.addHelpOption();
//    parser.addPositionalArgument("infile", "Input .obj file path");

//    parser.process(a);

//    const QStringList args = parser.positionalArguments();
//    if(args.size() < 1) {
//        cerr << "Error: Wrong number of arguments" << endl;
//        a.exit(1);
//        return 1;
//    }
//    QString infile = args[0];
    MainWindow w;
    srand (static_cast <unsigned> (time(0)));
    // We cannot use w.showFullscreen() here because on Linux that creates the
    // window behind all other windows, so we have to set it to fullscreen after
    // it has been shown. 
    w.show();
//    w.setWindowState(w.windowState() | Qt::WindowFullScreen); // Comment out this line to have a windowed 800x600 game on startup.

    return a.exec();
}

