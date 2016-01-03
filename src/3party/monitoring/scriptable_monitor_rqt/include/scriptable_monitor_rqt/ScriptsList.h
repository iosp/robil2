/********************************************************************************
** Form generated from reading UI file 'ScriptsList.ui'
**
** Created: Tue Dec 3 11:06:30 2013
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef SCRIPTSLIST_H
#define SCRIPTSLIST_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTreeView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ScriptsForm
{
public:
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *addScriptButton;
    QPushButton *refreshButton;
    QTreeView *scriptsTableView;

    void setupUi(QWidget *ScriptsForm)
    {
        if (ScriptsForm->objectName().isEmpty())
            ScriptsForm->setObjectName(QString::fromUtf8("ScriptsForm"));
        ScriptsForm->resize(578, 443);
        gridLayout_2 = new QGridLayout(ScriptsForm);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 3, 1, 1, 1);

        addScriptButton = new QPushButton(ScriptsForm);
        addScriptButton->setObjectName(QString::fromUtf8("addScriptButton"));

        gridLayout->addWidget(addScriptButton, 3, 3, 1, 1);

        refreshButton = new QPushButton(ScriptsForm);
        refreshButton->setObjectName(QString::fromUtf8("refreshButton"));

        gridLayout->addWidget(refreshButton, 3, 0, 1, 1);

        scriptsTableView = new QTreeView(ScriptsForm);
        scriptsTableView->setObjectName(QString::fromUtf8("scriptsTableView"));

        gridLayout->addWidget(scriptsTableView, 0, 0, 1, 4);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);


        retranslateUi(ScriptsForm);

        QMetaObject::connectSlotsByName(ScriptsForm);
    } // setupUi

    void retranslateUi(QWidget *ScriptsForm)
    {
        ScriptsForm->setWindowTitle(QApplication::translate("ScriptsForm", "Scripts", 0, QApplication::UnicodeUTF8));
        addScriptButton->setText(QApplication::translate("ScriptsForm", "Add script", 0, QApplication::UnicodeUTF8));
        refreshButton->setText(QApplication::translate("ScriptsForm", "Refresh", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ScriptsForm: public Ui_ScriptsForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // SCRIPTSLIST_H
