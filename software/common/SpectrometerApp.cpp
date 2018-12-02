/*
    SpectrometerApp.cpp - mainform class for Spectrometer QT application sample

    Copyright 2017-2018 Alexey Danilchenko

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

#include "SpectrometerApp.h"

#define APP_VERSION " v1.0"

#define MAIN_TITLE APP_NAME APP_VERSION

#define STATE_SECTION "Saved State"

#if defined( Q_OS_MACX )
#define BUNDLE_ID CFSTR("SpectrometerApp")
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
//    SpectrometerApp class
// --------------------------------------------------------
SpectrometerApp::SpectrometerApp(QWidget *parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags), overrideCursorSet(false),
      m_spectron(), m_pAPI(ParticleAPI::instance()), m_specSeries(0),
      m_axisX(0), m_axisY(0), ignoreUiUpdates(false)
{
    ui.setupUi(this);

    // buttons
    connect(ui.btnLogin, SIGNAL(clicked()), this, SLOT(login()));
    connect(ui.btnMeasure, SIGNAL(clicked()), this, SLOT(measure()));
    connect(ui.btnMeasureBlack, SIGNAL(clicked()), this, SLOT(measureBlack()));
    connect(ui.btnMeasSat, SIGNAL(clicked()), this, SLOT(measureSaturation()));

    // comboboxes
    connect(ui.cboxAdcRef, SIGNAL(currentIndexChanged(int)), this, SLOT(setADCRef(int)));
    connect(ui.cboxGain, SIGNAL(currentIndexChanged(int)), this, SLOT(setGain(int)));
    connect(ui.cboxUnits, SIGNAL(currentIndexChanged(int)), this, SLOT(setUnits(int)));
    connect(ui.cboxMeasResultType, SIGNAL(currentIndexChanged(int)), this, SLOT(setMeasResultType(int)));
    connect(ui.cboxMeasType, SIGNAL(currentIndexChanged(int)), this, SLOT(setMeasType(int)));

    setWindowTitle(MAIN_TITLE);

    // setup chart
    ui.wChart->chart()->setTheme(QChart::ChartThemeDark);
    ui.wChart->chart()->legend()->hide();

    m_specSeries = new QLineSeries();
	m_specSeries->clear();
    m_specSeries->append(340,0);
    m_specSeries->append(850,0);
    ui.wChart->chart()->addSeries(m_specSeries);
    m_axisX = new QValueAxis();
    m_axisY = new QValueAxis();
    m_axisX->setRange(340.0,850.0);
    m_axisX->setTickCount((850-340)/30+1);
    m_axisX->setLabelFormat("%d");
    m_axisY->setRange(0, 1.0);
    m_axisY->setTickCount(11);
    ui.wChart->chart()->setAxisX(m_axisX, m_specSeries);
    ui.wChart->chart()->setAxisY(m_axisY, m_specSeries);
}


SpectrometerApp::~SpectrometerApp()
{
}
void SpectrometerApp::updateAxis(SpectronDevice::TMeasType measType)
{
    if (measType == SpectronDevice::MEASURE_VOLTAGE)
    {
        m_specSeries->clear();
        m_specSeries->append(340,0);
        m_specSeries->append(850,0);
        double roundVolt = ceil(m_spectron.getSatVoltage()/0.5)*0.5;
        
        m_axisY->setRange(0, roundVolt);
        m_axisY->setTickCount(roundVolt/0.25 + 1);
    }
    else if (m_spectron.getMeasureType() == SpectronDevice::MEASURE_VOLTAGE)
    {
        m_specSeries->clear();
        m_specSeries->append(340,0);
        m_specSeries->append(850,0);
        m_axisY->setRange(0, 1.0);
        m_axisY->setTickCount(11);
    }
}

void SpectrometerApp::setADCRef(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));

    if (m_spectron.isConnected())
        m_spectron.setADCReference((SpectronDevice::TAdcRef)idx);
    restoreOverrideCursor();
}

void SpectrometerApp::setGain(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (m_spectron.getMeasureType() == SpectronDevice::MEASURE_VOLTAGE
            && idx != m_spectron.getGain())
            updateAxis(m_spectron.getMeasureType());
        m_spectron.setGain((SpectronDevice::TGain)idx);
    }

    restoreOverrideCursor();
}

void SpectrometerApp::setUnits(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
        if (ui.cboxUnits->currentIndex() == 1)
            ui.spbIntegration->setValue(m_spectron.getIntegTime()/1000);
        else
            ui.spbIntegration->setValue(m_spectron.getIntegTime());
    restoreOverrideCursor();
}

void SpectrometerApp::setMeasResultType(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (m_spectron.getMeasureType() != idx)
            updateAxis((SpectronDevice::TMeasType)idx);
        m_spectron.setMeasureType((SpectronDevice::TMeasType)idx);
    }

    restoreOverrideCursor();
}

void SpectrometerApp::setMeasType(int idx)
{
    if (ignoreUiUpdates)
        return;

    bool intEnable = idx==0;
    
    ui.lblIntegration->setEnabled(intEnable);
    ui.spbIntegration->setEnabled(intEnable);
    ui.cboxUnits->setEnabled(intEnable);
    ui.btnMeasureBlack->setEnabled(intEnable);
    ui.btnMeasSat->setEnabled(intEnable);

    if (!m_spectron.supportsGain())
    {
        ui.lblMeasureTime->setEnabled(intEnable);
        ui.spbMeasureTime->setEnabled(intEnable);
        ui.lblMeasureUnits->setEnabled(intEnable);
    }
}

void SpectrometerApp::login()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    // login with user/password - get non expiring auth token.
    if (m_pAPI.login(ui.edtUser->text(), ui.edtPwd->text(), 0))
    {
        TParticleDeviceList devices;
        if (m_pAPI.getAllMatchingDevices(devices, QString(""), true)
            && !devices.empty())
        {
            // iterate the connected device list and find internal
            // devices for spectrometer
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
                        ignoreUiUpdates = true;
                        updateAxis(m_spectron.getMeasureType());
                        ui.cboxAdcRef->setCurrentIndex(m_spectron.getADCReference());
                        ui.cboxMeasResultType->setCurrentIndex(m_spectron.getMeasureType());
                        if (m_spectron.supportsGain())
                        {
                            ui.lblGain->setEnabled(true);
                            ui.cboxGain->setEnabled(true);
                            ui.cboxGain->setCurrentIndex(m_spectron.getGain());
                            ui.lblMeasureTime->setEnabled(false);
                            ui.spbMeasureTime->setEnabled(false);
                            ui.lblMeasureUnits->setEnabled(false);
                            ui.lblSpectrometer->setText("Spectrometer: Hamamatsu C12666MA  ");
                        }
                        else
                        {
                            ui.lblGain->setEnabled(false);
                            ui.cboxGain->setEnabled(false);
                            ui.lblMeasureTime->setEnabled(true);
                            ui.spbMeasureTime->setEnabled(true);
                            ui.lblMeasureUnits->setEnabled(true);
                            ui.lblSpectrometer->setText("Spectrometer: Hamamatsu C12880MA  ");
                        }
                        if (ui.cboxUnits->currentIndex() == 1)
                            ui.spbIntegration->setValue(m_spectron.getIntegTime()/1000);
                        else
                            ui.spbIntegration->setValue(m_spectron.getIntegTime());
                        ignoreUiUpdates = false;
                    }
                }
                ++it;
            }

        }
    }
    else
        ui.txtCSV->setPlainText(m_pAPI.getLastError());

    restoreOverrideCursor();
}

void SpectrometerApp::measure()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (ui.cboxMeasType->currentIndex() == 0)
        {
            int integTime = ui.spbIntegration->value();
            int setIntegTime = m_spectron.getIntegTime();

            if (ui.cboxUnits->currentIndex() == 1)
            {
                integTime *= 1000;
                setIntegTime /= 1000;
            }

            // set integration time if changed
            if (setIntegTime != ui.spbIntegration->value())
                m_spectron.setIntegrationTime(integTime);

            int measureTime = 0;
            if (!m_spectron.supportsGain())
                measureTime = ui.spbMeasureTime->value()*1000;

           if (m_spectron.measure(measureTime))
                readMeasurement();
            else
                ui.txtCSV->setPlainText(m_pAPI.getLastError());
        }
        else
        {
            if (m_spectron.measureAuto(
                  (SpectronDevice::TAutoType)(ui.cboxMeasType->currentIndex()-1)))
            {
                ui.cboxAdcRef->setCurrentIndex(m_spectron.getADCReference());
                ui.cboxGain->setCurrentIndex(m_spectron.getGain());
                if (ui.cboxUnits->currentIndex() == 1)
                    ui.spbIntegration->setValue(m_spectron.getIntegTime()/1000);
                else
                    ui.spbIntegration->setValue(m_spectron.getIntegTime());
                updateAxis(m_spectron.getMeasureType());
                readMeasurement();
            }
            else
                ui.txtCSV->setPlainText(m_pAPI.getLastError());
        }
    }

    restoreOverrideCursor();
}

void SpectrometerApp::measureBlack()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (ui.cboxMeasType->currentIndex() == 0)
        {
            int integTime = ui.spbIntegration->value();
            int setIntegTime = m_spectron.getIntegTime();

            if (ui.cboxUnits->currentIndex() == 1)
            {
                integTime *= 1000;
                setIntegTime /= 1000;
            }

            if (setIntegTime != ui.spbIntegration->value())
                m_spectron.setIntegrationTime(integTime);

            int measureTime = 0;
            if (!m_spectron.supportsGain())
                measureTime = ui.spbMeasureTime->value()*1000;

           if (m_spectron.measureBlack(measureTime))
                readMeasurement();
            else
                ui.txtCSV->setPlainText(m_pAPI.getLastError());
        }
    }

    restoreOverrideCursor();
}

void SpectrometerApp::measureSaturation()
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
        m_spectron.measureSaturation();
    restoreOverrideCursor();
}

void SpectrometerApp::saveCSV()
{
}

void SpectrometerApp::readMeasurement()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        // populate last measurement
        m_specSeries->clear();
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
        ui.lblMaxValue->setText(QString("%1 at %2 nm")
                .arg(maxVal, 0, 'F', 4)
                .arg(m_spectron.getWavelength(maxIdx), 0, 'F', 2));

        ui.txtCSV->setPlainText(csv);
    }

    restoreOverrideCursor();
}


void SpectrometerApp::closeEvent(QCloseEvent *event)
{
}

int SpectrometerApp::showMessage(const QString& title,
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

void SpectrometerApp::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
}


void SpectrometerApp::updateWidgets()
{
    QString title(MAIN_TITLE);

    setWindowTitle(title);
}

// -------------------------------------------------------------------------
//   Event slots
// -------------------------------------------------------------------------
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

    SpectrometerApp specMain;
    specMain.show();
    return app.exec();
}
