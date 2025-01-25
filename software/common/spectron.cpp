/*
    spectron.cpp - mainform class for Spectron QT application

    Copyright 2017-2019 Alexey Danilchenko

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3, or (at your option)
    any later version with ADDITION (see below).

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, 51 Franklin Street - Fifth Floor, Boston,
    MA 02110-1301, USA.
*/
#include <QPalette>
#include <QProxyStyle>
#include <QThread>
#include <QStyle>
#include <QStyleFactory>
#include <QString>

#include <math.h>

#include "spectron.h"
#include "spectron_fit.h"

#define APP_VERSION " v1.2"

#define MAIN_TITLE APP_NAME APP_VERSION

#define STATE_SECTION "Saved State"

#if defined( Q_OS_MACX )
#define BUNDLE_ID CFSTR("Spectron")
#if QT_VERSION >= 0x050000
#include <QtPlugin>
Q_IMPORT_PLUGIN (QCocoaIntegrationPlugin);
#endif
#endif

#if defined( Q_OS_WIN )
//#include <omp.h>
#if defined(_QT_STATIC_) && QT_VERSION >= 0x050000
#include <QtPlugin>
Q_IMPORT_PLUGIN (QWindowsIntegrationPlugin);
#endif
#endif

// --------------------------------------------------------
//    static data
// --------------------------------------------------------

// --------------------------------------------------------
//    helper functions
// --------------------------------------------------------

// --------------------------------------------------------
//    Spectron class
// --------------------------------------------------------
Spectron::Spectron(QWidget *parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags),
      unsavedChanges(false), overrideCursorSet(false),
      m_spectron(), m_motor(), m_lightSource(0),
      m_pAPI(ParticleAPI::instance()), m_specSeries(0),
      ignoreUiUpdates(false)
{
    //language = QLocale::system().language();

    ui.setupUi(this);

    // buttons
    connect(ui.btnLogin, SIGNAL(clicked()), this, SLOT(login()));
    connect(ui.btnMeasure, SIGNAL(clicked()), this, SLOT(measure()));
    connect(ui.btnMeasureAuto, 
            &QPushButton::clicked, 
            [this]() { measureAuto(SpectronDevice::AUTO_FOR_SET_REF);});
    connect(ui.btnMeasureAutoMinInt, 
            &QPushButton::clicked,
            [this]() { measureAuto(SpectronDevice::AUTO_ALL_MIN_INTEG);});
    connect(ui.btnMeasureAutoMaxRange, 
            &QPushButton::clicked, 
            [this]() { measureAuto(SpectronDevice::AUTO_ALL_MAX_RANGE);});
    connect(ui.btnMeasureBlack, SIGNAL(clicked()), this, SLOT(measureBlack()));
    connect(ui.btnMeasureCombSpectrum, SIGNAL(clicked()), this, SLOT(measureSpectrum()));
    connect(ui.btnWavelength, SIGNAL(clicked()), this, SLOT(setWavelength()));

    // comboboxes
    connect(ui.cboxAdcRef, SIGNAL(currentIndexChanged(int)), this, SLOT(setADCRef(int)));
    connect(ui.cboxGain, SIGNAL(currentIndexChanged(int)), this, SLOT(setGain(int)));

    setWindowTitle(MAIN_TITLE);

    // init data
    init();

    // setup chart
    ui.wChart->chart()->setTheme(QChart::ChartThemeDark);
    //ui.wChart->chart()->removeAllSeries();
    ui.wChart->chart()->legend()->hide();

    m_specSeries = new QLineSeries();
	m_specSeries->clear();
    m_specSeries->append(340,0);
    m_specSeries->append(850,0);
    ui.wChart->chart()->addSeries(m_specSeries);
    m_gaussSeries = new QLineSeries();
    m_gaussSeries->clear();
    m_gaussSeries->setColor(QColor(255, 0, 0, 127));
    ui.wChart->chart()->addSeries(m_gaussSeries);
    QValueAxis* axisX = new QValueAxis();
    QValueAxis* axisY = new QValueAxis();
    axisX->setRange(340.0,850.0);
    axisX->setTickCount((850-340)/30+1);
    axisX->setLabelFormat("%d");
    axisY->setRange(0, 1.1);
    axisY->setTickCount(12);
    ui.wChart->chart()->setAxisX(axisX, m_gaussSeries);
    ui.wChart->chart()->setAxisY(axisY, m_gaussSeries);
    ui.wChart->chart()->setAxisX(axisX, m_specSeries);
    ui.wChart->chart()->setAxisY(axisY, m_specSeries);
    ui.wChart->setRubberBand(QChartView::HorizontalRubberBand);
}


Spectron::~Spectron()
{
    delete m_lightSource;
}
void Spectron::setADCRef(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));

    if (m_spectron.isConnected())
        m_spectron.setADCReference((SpectronDevice::TAdcRef)idx);
    restoreOverrideCursor();
}

void Spectron::setGain(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
        m_spectron.setGain((SpectronDevice::TGain)idx);
    restoreOverrideCursor();
}

void Spectron::login()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    // login with user/password - get non expiring auth token.
    if (/*!m_spectron.isConnected() && */m_pAPI.login(ui.edtUser->text(), ui.edtPwd->text(), 0))
    {
        TParticleDeviceList devices;
        if (m_pAPI.getAllMatchingDevices(devices, QString(""), true)
            && !devices.empty())
        {
            // iterate the connected device list and initialise
            // internal devices for spectrometer, motor and light source
            TParticleDeviceList::iterator it = devices.begin();
            while (it != devices.end())
            {
                if (it->refresh())
                {
                    QString boardType = it->getVariableValue("BOARD_TYPE").toString();
                    if (boardType == QString("SPEC2_SPECTROMETER"))
                    {
                        // spectrometer device
                        m_spectron = *it;
                        m_spectron.refresh();
                        if (m_spectron.getMeasureType() != SpectronDevice::MEASURE_ABSOLUTE)
                            m_spectron.setMeasureType(SpectronDevice::MEASURE_ABSOLUTE);
                        ui.txtLog->setPlainText(m_spectron.getLastResponse());
                        ignoreUiUpdates = true;
                        ui.cboxAdcRef->setCurrentIndex(m_spectron.getADCReference());
                        ui.cboxGain->setCurrentIndex(m_spectron.getGain());
                        ui.spbIntegration->setValue(m_spectron.getIntegTime());
                        ignoreUiUpdates = false;
                    }
                    else if (boardType == QString("SPEC2_MOTOR"))
                    {
                        // get motor device
                        m_motor = *it;
                        m_motor.refresh();
                        ui.spbWavelength->setValue(m_motor.getCurrentPos());
                        ui.spbWavelength->setMaximum(m_motor.getMaxWavelength());
                        ui.spbWavelength->setMinimum(m_motor.getMinWavelength());
                    }
                    else if (boardType == QString("SPEC2_XENON"))
                    {
                        if (!m_lightSource)
                        {
                            m_lightSource = new SpecXenonDevice(*it);
                            m_lightSource->refresh();
                        }
                    }
                    else if (boardType == QString("SPEC2_LED"))
                    {
                        if (!m_lightSource)
                        {
                            m_lightSource = new SpecLEDDevice(*it);
                            m_lightSource->refresh();
                        }
                    }
                }
                ++it;
            }

        }
    }
    else
        ui.txtLog->setPlainText(m_pAPI.getLastError());
    
    restoreOverrideCursor();
}

void Spectron::measure()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (m_lightSource)
            m_lightSource->trigger(ui.spbMeasureTime->value() + 1000);
        if (m_spectron.getIntegTime() != ui.spbIntegration->value())
            m_spectron.setIntegrationTime(ui.spbIntegration->value());
        if (m_spectron.measure(ui.spbMeasureTime->value()*1000))
            readMeasurement();
		else
			ui.txtLog->setPlainText(m_pAPI.getLastError());
    }

    restoreOverrideCursor();
}

void Spectron::measureAuto(SpectronDevice::TAutoType autoType)
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (m_lightSource)
            m_lightSource->trigger(5000);
        if (m_spectron.measureAuto(autoType))
        {
            readMeasurement();
            ui.cboxAdcRef->setCurrentIndex(m_spectron.getADCReference());
            ui.cboxGain->setCurrentIndex(m_spectron.getGain());
            ui.spbIntegration->setValue(m_spectron.getIntegTime());
        }
		else
			ui.txtLog->setPlainText(m_pAPI.getLastError());
    }

    restoreOverrideCursor();
}

void Spectron::measureBlack()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (m_spectron.getIntegTime() != ui.spbIntegration->value())
            m_spectron.setIntegrationTime(ui.spbIntegration->value());
        if (m_spectron.measureBlack(ui.spbMeasureTime->value()*1000, true))
            readMeasurement();
    }

    restoreOverrideCursor();
}

void Spectron::measureSpectrum()
{
    if (!m_motor.isConnected() || !m_spectron.isConnected())
        return;
    
    setOverrideCursor(QCursor(Qt::WaitCursor));
    int savedWavelength = m_motor.getCurrentPos();
    m_spectron.setIntegrationTime(ui.spbIntegration->value());
    m_specSeries->clear();
    QString csv = "Wavelength,Measurement\n";
    for (int wavelength = 380; wavelength<=740; wavelength+=5)
    {
        // pause before next itertion
        if (wavelength > 380)
            QThread::sleep(10);

        // loop through spectral range
        m_motor.moveToWavelength(wavelength);
        if (m_lightSource)
            m_lightSource->trigger(ui.spbMeasureTime->value() + 500);
        if (m_spectron.measure(ui.spbMeasureTime->value()*1000))
        {
            // populate last measurement
            float maxVal = 0;
            for (int i=0; i<m_spectron.totalPixels(); i++)
            {
                if (m_spectron.getLastMeasurement(i) > maxVal)
                    maxVal = m_spectron.getLastMeasurement(i);
            }
            m_specSeries->append(wavelength, maxVal);
            csv.append(QString("%1,%2\n").arg(wavelength).arg(maxVal));
        }
		else
			ui.txtLog->setPlainText(m_pAPI.getLastError());
    }
    
    // restore wavelength
    m_motor.moveToWavelength(savedWavelength);
    
    ui.txtLog->setPlainText(csv);
    
    restoreOverrideCursor();
}

void Spectron::readMeasurement()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        // populate last measurement
        m_specSeries->clear();
        m_gaussSeries->clear();
        QString csv = "Wavelength,Measurement\n";
        double maxVal = 0, minVal = 1;
        int maxIdx = 0;
        for (int i=0; i<m_spectron.totalPixels(); i++)
        {
            csv.append(QString("%1,%2\n").arg(m_spectron.getWavelength(i)).arg(m_spectron.getLastMeasurement(i)));
            if (m_spectron.getLastMeasurement(i) > maxVal)
            {
                maxVal = m_spectron.getLastMeasurement(i);
                maxIdx = i;
            }
            else if (m_spectron.getLastMeasurement(i) < minVal)
                minVal = m_spectron.getLastMeasurement(i);
            m_specSeries->append(m_spectron.getWavelength(i), m_spectron.getLastMeasurement(i));
        }

        // calculate true peak - first find peak points that almost 
        // go down to the tail
        int fromIdx = maxIdx, toIdx = maxIdx;
        while (fromIdx > 0 && 
               m_spectron.getLastMeasurement(fromIdx-1) < m_spectron.getLastMeasurement(fromIdx) &&
               m_spectron.getLastMeasurement(fromIdx-1)-minVal > 0.2*(maxVal-minVal))
            --fromIdx;
        while (toIdx+1 < m_spectron.totalPixels() && 
               m_spectron.getLastMeasurement(toIdx+1) < m_spectron.getLastMeasurement(toIdx) &&
               m_spectron.getLastMeasurement(toIdx+1)-minVal > 0.2*(maxVal-minVal))
            ++toIdx;

        // prepare peak parts for fitting
        std::vector<double> valX(toIdx-fromIdx+1);
        std::vector<double> valY(toIdx-fromIdx+1);
        
        for (int i=fromIdx; i<=toIdx; i++)
        {
            valX[i-fromIdx] = m_spectron.getWavelength(i);
            valY[i-fromIdx] = m_spectron.getLastMeasurement(i)-minVal;
        }

        double m=0, A=0, q=0;
        int iter=0;
        if (iter=fitGaussIterative(valX, valY, m, q, A, 1))
        {
            // add Gauss series
            for (int wl=340; wl<=850; wl++)
            {
                double gaussVal = minVal + A*exp(-1*(wl-m)*(wl-m)/(2*q*q));
                m_gaussSeries->append(wl, gaussVal);
            }
            ui.lblMaxValue->setText(QString("%1, iter=%2").arg(m, 0, 'F', 6).arg(iter));
        }
        else
            ui.lblMaxValue->setText(
                QString("%1").arg(m_spectron.getWavelength(maxIdx), 0, 'F', 6));

        ui.txtLog->setPlainText(csv);
    }

    restoreOverrideCursor();
}

void Spectron::setWavelength()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_motor.isConnected())
    {
        if (m_motor.moveToWavelength(ui.spbWavelength->value()))
        {
            ui.spbWavelength->setValue(m_motor.getCurrentPos());
        }
    }

    restoreOverrideCursor();
}

bool Spectron::checkUnsavedAndSave()
{
    bool okToProceed = true;

    if (unsavedChanges)
    {
        int dlgRes = showMessage(
                        tr("Warning"),
                        tr("The remap has been modified!"),
                        tr("Do you want to save your changes?"),
                        QMessageBox::Question,
                        QMessageBox::Save
                            | QMessageBox::Discard
                            | QMessageBox::Cancel);

        if (dlgRes == QMessageBox::Save)
            okToProceed = true;
        else
            okToProceed = dlgRes==QMessageBox::Discard;

        if (okToProceed)
            unsavedChanges = false;
    }

    return okToProceed;
}


void Spectron::closeEvent(QCloseEvent *event)
{
}

/*
void Spectron::retranslate()
{
    if (language == QLocale::Russian)
    {
        translator.load(QStringLiteral(":/MainForm/dcs_remap_ru.qm"));
        QApplication::installTranslator(&translator);
    }
    else
       QApplication::removeTranslator(&translator);

    ui.retranslateUi(this);
    ui.cboxZoomLevel->setItemText(0, tr("Fit to Window"));
    ui.rawImage->retranslate();
    updateRawStats();
    updateDefectStats();
    updateThresholdStats(C_ALL);
    updateWidgets();
    ui.cboxZoomLevel->updateGeometry();
}

void Spectron::setRussianLanguage()
{
    language = QLocale::Russian;
    retranslate();
}

void Spectron::setEnglishLanguage()
{
    language = QLocale::English;
    retranslate();
}
*/
void Spectron::init()
{
}

int Spectron::showMessage(const QString& title,
                          const QString& msgText,
                          const QString& informativeText,
                          QMessageBox::Icon icon,
                          QMessageBox::StandardButtons buttons,
                          QMessageBox::StandardButton defButton)
{
    restoreOverrideCursor();

    QMessageBox msgBox(icon,
                       title,
                       msgText,
                       buttons);
    msgBox.setInformativeText(informativeText);
    msgBox.setDefaultButton(defButton);

    return msgBox.exec();
}

void Spectron::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
}


void Spectron::updateWidgets()
{

    QString title(MAIN_TITLE);

    setWindowTitle(title);

}

// -------------------------------------------------------------------------
//   Event slots
// -------------------------------------------------------------------------

void Spectron::updateStats()
{
}

void Spectron::updateStatus()
{
}

void Spectron::help()
{
    QDir dir(QApplication::applicationDirPath());

#ifdef Q_OS_MACX
    dir.cdUp();
#endif
/*
    if (dir.cd("help"))
        if (language == QLocale::Russian)
            QDesktopServices::openUrl(QUrl::fromLocalFile(dir.filePath("help_ru.html")));
        else
            QDesktopServices::openUrl(QUrl::fromLocalFile(dir.filePath("help_en.html")));
*/
}

void Spectron::about()
{
//    About about(language);

//    about.exec();
}

// -------------------------------------------------------------------------
//   Fusion proxy style to disable stupid QStyle::SH_ComboBox_Popup
// -------------------------------------------------------------------------
class DCSProxyStyle : public QProxyStyle
{
public:
    DCSProxyStyle(QStyle *style): QProxyStyle(style) {}

    int styleHint(StyleHint hint, const QStyleOption *option, const QWidget *widget, QStyleHintReturn *returnData) const
    {
        if (hint == QStyle::SH_ComboBox_Popup)
        {
            return 0;
        }
        return QProxyStyle::styleHint(hint, option, widget, returnData);
    }

    void polish (QWidget *w)
    {
#ifdef Q_OS_MACX
        QMenu* mn = qobject_cast<QMenu*>(w);
        if (!mn && !w->testAttribute(Qt::WA_MacNormalSize))
            w->setAttribute(Qt::WA_MacSmallSize);
#endif
    }
};

// -------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

#if QT_VERSION >= 0x050000
	app.setStyle(new DCSProxyStyle(QStyleFactory::create("fusion")));
#endif

    QPalette palette;
    palette.setColor(QPalette::Window, QColor(83,83,83));
    palette.setColor(QPalette::WindowText, Qt::white);
    palette.setColor(QPalette::Base, QColor(63,63,63));
    palette.setColor(QPalette::AlternateBase, QColor(83,83,83));
    palette.setColor(QPalette::ToolTipBase, QColor(94,180,255));
    palette.setColor(QPalette::ToolTipText, Qt::black);
    palette.setColor(QPalette::Text, Qt::white);
    palette.setColor(QPalette::Button, QColor(83,83,83));
    palette.setColor(QPalette::ButtonText, Qt::white);
    palette.setColor(QPalette::BrightText, Qt::red);
    palette.setColor(QPalette::Highlight, QColor(51,153,255));
    palette.setColor(QPalette::HighlightedText, Qt::black);

    palette.setColor(QPalette::Disabled, QPalette::WindowText, Qt::gray);
    palette.setColor(QPalette::Disabled, QPalette::ButtonText, Qt::gray);

    app.setPalette(palette);


    Spectron specMain;
    specMain.show();
    return app.exec();
}
